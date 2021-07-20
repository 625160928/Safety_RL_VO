import math
import time

import numpy as np
from matplotlib.ticker import MultipleLocator
import matplotlib.pyplot as plt

from env import Env

from parking_planning import Planning

import rospy

import carla_info_setting
from car_parking.controller import stanley_method, pure_pursuit_method
from controller import pid_lateral_controller

class Tracking():
    def __init__(self, plan,model,draw=False):

        #设置泊车算法
        self.plan=plan

        # 认为接近这个距离就到终点
        self.goal_arrive_distance = 0.5

        #车辆目标速度
        self.tar_speed = 1  # [m/s]

        #是否重规划
        self.replanning=True

        #重规划距离
        self.replanning_distance = 0.5

        # 速度P控制器系数 1
        self.Speed_Kp = 1
        # 时间间隔，单位：s 0.01
        self.dt=self.plan.env.step
        # 车辆轴距，单位：m
        self.L = self.plan.env.wheel_dis


        #  限制车轮转角
        self.car_steer_limit = math.pi / 6

        #期望车辆的速度
        self.target_car_speed = 0
        self.target_car_steering = 0

        self.model = model
        self.mode = model.mode

        self.front_move_control= stanley_method.StanleyControl(self.L, self.dt, self.car_steer_limit)

        # self.back_move_control= pure_pursuit_method.PurePursuitControl(self.L, self.dt, self.car_steer_limit)

        #K_P=0.08, K_D=0.01, K_I=0.0

        self.back_move_control=pid_lateral_controller.PIDLateralController(L=self.L, dt=self.dt,car_steer_limit= self.car_steer_limit,
                                                                           K_P=0.09, K_D=0.01, K_I=0.0)


        self.draw_pic =draw

        self.__planning_route=None

    #获取当前跟踪的路径
    def get_planning_route(self):
        return self.__planning_route

    #速度控制器
    def __PControl(self, target, current):
        return self.Speed_Kp * (target - current)

    #求两点之间的距离
    def __distance(self, x1, y1, x2, y2):
        return np.hypot((x1-x2),(y1-y2))

    #去除路径中的重复点
    def __route_drop_repeat_point(self, route_x, route_y, route_theta):
        new_route_x=[route_x[0]]
        new_route_y=[route_y[0]]
        if route_theta[0]>math.pi:
            route_theta[0]-= 2 * math.pi
        if route_theta[0] < -math.pi:
            route_theta[0] += 2 * math.pi
        new_route_theta=[route_theta[0]]
        for i in range(1, len(route_x)):
            if (route_x[i]!=route_x[i - 1] or route_y[i]!=route_y[i - 1] or route_theta[i]!=route_theta[i - 1]):
                new_route_x.append(route_x[i])
                new_route_y.append(route_y[i])
                if route_theta[i]>math.pi:
                    route_theta[i]-= 2 * math.pi
                if route_theta[i] < -math.pi:
                    route_theta[i] += 2 * math.pi

                new_route_theta.append(route_theta[i])
        return new_route_x,new_route_y,new_route_theta

    # 将路径分段，尽量将正向跟踪和反向跟踪的部分分开，
    # 实际上分开效果太好,有时同样是正向的都会分开
    def __route_device(self, route_x, route_y, rtheta):
        #记录分割点
        device_points = [0]

        old = None
        #对所有点进行遍历，通过分析当前点的方向与到下一点的实际位移方向来判断是否改变了移动方向
        for i in range(len(route_x) - 1):

            dir_x = route_x[i + 1] - route_x[i]
            dir_y = route_y[i + 1] - route_y[i]
            if dir_x != 0:
                new = math.cos(rtheta[i]) / dir_x
            elif dir_y != 0:
                new = math.sin(rtheta[i]) / dir_y
            else:
                continue

            if old is None:
                old = new
            else:
                if old * new < 0:
                    device_points.append(i)
                old = new

        device_points.append(len(route_x) - 2)

        #通过分割点集，将路径实际分割并返回结果
        ans_x = []
        ans_y = []
        ans_theta = []
        for i in range(1, len(device_points)):
            tmp_x = []
            tmp_y = []
            tmp_theta = []
            for j in range(device_points[i - 1], device_points[i] + 1):
                tmp_x.append(route_x[j])
                tmp_y.append(route_y[j])
                tmp_theta.append(rtheta[j])
            ans_x.append(tmp_x)
            ans_y.append(tmp_y)
            ans_theta.append(tmp_theta)
        return ans_x, ans_y, ans_theta


    #有改变，求点到路径最短距离
    def __min_distance(self, x, y, route_x, route_y):
        dx = [x - icx for icx in route_x]
        dy = [y - icy for icy in route_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        return min(d)



    def __direction_check(self, now_x, now_y, now_theta, route_x, route_y):
        # 通过车辆当前角度与需要移动的方向计算是不是要换档从前进到后退又或者取反
        dir_x = route_x[-1] - now_x
        dir_y = route_y[-1] - now_y
        cos_theta = (dir_x * math.cos(now_theta) + dir_y * math.sin(now_theta)) / \
                    np.hypot(dir_x, dir_y)

        # 计算出来的结果可能超界
        if cos_theta > 1:
            cos_theta = 1
        if cos_theta < -1:
            cos_theta = -1
        theta_pd = math.acos(cos_theta)
        new_dir = 1
        if theta_pd > math.pi / 2:
            new_dir = -1
        return new_dir

    #stanley+pure pursuit结合合的的tracking跟踪
    def __tracking(self, start_position_x, start_position_y, start_position_theta,
                   start_car_speed, route_x, route_y, route_theta, tar_speed):
        now_x = start_position_x
        now_y = start_position_y
        now_theta = start_position_theta
        now_car_speed=start_car_speed

        target_speed=tar_speed

        v_list = []
        w_list = []
        target_ind = 0
        down_speed=False

        # count=0

        while True:
            # 当跟踪到最后一个点就推出
            if len(route_x)-1==target_ind:
                break



            if rospy.has_param("/parking_planning/parking_end"):
                if  rospy.get_param("/parking_planning/parking_end")==0:
                    break

            #　当要接近终点时减速
            if  self.__distance(now_x, now_y, route_x[len(route_x) - 1], route_y[len(route_y) - 1])<0.5 and down_speed==False:
                target_speed=target_speed/2
                down_speed=True



            #通过ｐ控制器获取当前期望加速度
            ai = self.__PControl(target_speed, now_car_speed)

            # 通过期望速度确定是正向还是反向跟踪
            if target_speed>=0:
                now_car_speed, w, target_ind=self.front_move_control.control(now_x, now_y, now_theta, now_car_speed,
                                                                             route_x, route_y, route_theta, target_ind,ai)
            else:
                now_car_speed, w, target_ind=self.back_move_control.control(now_x, now_y, now_theta, now_car_speed,
                                                                             route_x, route_y, route_theta, target_ind,ai)


            self.model.update(now_car_speed, w)
            now_x, now_y, now_theta, now_car_speed,_ = self.model.get_infomation()

            v_list.append(now_car_speed)
            w_list.append(w)

            #重规划部分
            if self.__replanning_check(now_x, now_y,now_theta, route_x, route_y,route_theta):
                # 计算路径
                try:
                    new_route_x, new_route_y, new_route_theta_r = self.plan.planning(now_x, now_y, now_theta)
                    if new_route_x != None :
                        if self.__min_distance(now_x, now_y, new_route_x, new_route_y)<self.replanning_distance:
                            print("replanning!")
                            return None,None,None,new_route_x, new_route_y, new_route_theta_r
                        else:
                            print("new route is not all right!")
                except Exception:
                    # print('当前位置无法重规划')
                    a=0

            if self.draw_pic==True:

                #画图
                plt.figure(num='Figure 1', dpi=70,)
                x_major_locator = MultipleLocator(1)
                y_major_locator = MultipleLocator(1)
                ax = plt.gca()
                #绘制全局路径
                global_route_x=[]
                global_route_y=[]
                for i in range(len(self.__planning_route)):
                    # print(self.planning_route[i])
                    global_route_x.append(self.__planning_route[i][0])
                    global_route_y.append(self.__planning_route[i][1])
                plt.plot(global_route_x,global_route_y,color=[0,1,1])
                #绘制车位后边缘
                real_parking_tail1_x,real_parking_tail1_y,_=self.plan.space_change.sim_to_real(self.plan.env.parking_w*2,-self.plan.env.parking_l,0)
                real_parking_tail2_x, real_parking_tail2_y, _ = self.plan.space_change.sim_to_real(self.plan.env.parking_w,
                                                                                                 -self.plan.env.parking_l,
                                                                                                0)
                plt.plot([real_parking_tail1_x,real_parking_tail2_x],[real_parking_tail1_y,real_parking_tail2_y])


                # ax为两条坐标轴的实例
                ax.xaxis.set_major_locator(x_major_locator)
                # 把x轴的主刻度设置为1的倍数
                ax.yaxis.set_major_locator(y_major_locator)
                #绘制车位前边缘
                plt.plot([self.plan.env.parking_left_front_point_x,self.plan.env.parking_right_front_point_x]
                         ,[self.plan.env.parking_left_front_point_y,self.plan.env.parking_right_front_point_y])

                #绘制车辆
                self.model.draw_car()
                plt.scatter(now_x,now_y)
                plt.scatter(route_x[target_ind],route_y[target_ind])
                plt.plot(route_x,route_y,color='black')
                road1_x,road1_y,_=self.plan.space_change.sim_to_real(0,self.plan.env.road_w,0)
                road2_x,road2_y,_=self.plan.space_change.sim_to_real(self.plan.env.parking_w*5,self.plan.env.road_w,0)

                plt.plot([road1_x,road2_x],[road1_y,road2_y])
                plt.pause(0.1)
                plt.clf()


        return now_x, now_y, now_theta, start_car_speed, v_list, w_list

    #是否重规划的判断条件，如果满足重规划的条件就返回True
    def __replanning_check(self,x,y,theta,rx,ry,rtheta):
        if self.replanning==False:
            return False

        if self.__min_distance(x, y, rx, ry) > self.replanning_distance:
            return True

        corners=self.model.get_corners()
        print(corners)



        return False

    #泊车跟踪
    def parking_tracking(self,  route_x, route_y, route_theta):

        replanning_mode = False

        planning_route=[]
        for i in range(len(route_x)):
            planning_route.append([route_x[i],route_y[i],route_theta[i]])
        self.__planning_route=planning_route


        if self.mode=="ros":
            rospy.set_param("/parking_planning/parking_end", 1)

        v_list = []
        w_list = []
        while (1):
            if self.replanning==True:
                if replanning_mode == True:
                    replanning_mode = False
                    self.model.stop()
                    print("----------------------replanning----------------------")

                    planning_route = []
                    for i in range(len(route_x)):
                        planning_route.append([route_x[i], route_y[i], route_theta[i]])
                        print([route_x[i], route_y[i], route_theta[i]])
                    self.__planning_route = planning_route

            nx, ny, ntheta = self.__route_drop_repeat_point(route_x, route_y, route_theta)
            dev_x, dev_y, dev_theta = self.__route_device(nx, ny, ntheta)

            self.model.update(0,0)
            now_x, now_y, now_theta, now_speed,_ =self.model.get_infomation()

            old_dir = 1

            for i in range(0, len(dev_x)):

                # print(i)
                # 通过车辆当前角度与需要移动的方向计算是不是要换档从前进到后退又或者取反
                new_dir=self.__direction_check(now_x, now_y, now_theta, dev_x[i], dev_y[i])

                #通过方向设置目标速度
                if new_dir==-1:
                    target_speed = -self.tar_speed
                else:
                    target_speed = self.tar_speed

                #如果之前一段路线与现在的路线反向，那么需要先停车再跟踪
                if old_dir != new_dir:
                    self.model.stop()
                    old_dir = new_dir

                now_x, now_y, now_theta, now_speed, v_tmp_list, w_tmp_list = self.__tracking(now_x, now_y, now_theta,
                                                                                             now_speed, dev_x[i],
                                                                                             dev_y[i], dev_theta[i],
                                                                                             target_speed)
                if self.replanning==True:
                    if now_x == None:
                        replanning_mode = True

                        route_x = now_speed
                        route_y = v_tmp_list
                        route_theta = w_tmp_list
                        break
                v_list += v_tmp_list
                w_list += w_tmp_list

            if replanning_mode == False:
                break


        self.model.stop()

        time.sleep(1)
        if self.mode=="ros":
            rospy.set_param("/parking_planning/parking_end", 0)

        return v_list,w_list



def __main_test_simulate():
    from car_parking.car_model import simulate_car_model
    #车辆实际坐标
    car_x=-37.131065
    car_y=-14.28638
    car_theta_r=-1.356844


    # 车位实际坐标
    real_parking_left_head = [-42.3, -3.1]
    real_parking_right_head = [-42.3, -6.1]

    # 车位数据，对于垂直停车，车位宽度由车位前侧左右两点决定，不需要输入
    parking_l = 7  # 魔改成7，实际标准是6
    road_w = 7
    road_l = 3 * parking_l

    # 进行规划所需要的车辆数据
    car_l = 3.6
    car_w = 1.7
    min_turning_radiu = 10
    wheel_dis = 2.4
    step = 0.3
    hou_xuan = 0.7
    parking_w = math.sqrt(math.pow(real_parking_left_head[0] - real_parking_right_head[0], 2) + math.pow(
        real_parking_left_head[1] - real_parking_right_head[1], 2))

    env0 = Env()
    env0.set_env_info(parking_l, parking_w, road_w, road_l, real_parking_left_head, real_parking_right_head)
    env0.set_car_info(car_l, car_w, min_turning_radiu, wheel_dis, hou_xuan, step)

    plan = Planning(env0)

    # sim_x,sim_y,sim_theta=plan.space_change.real_to_sim(car_x, car_y, car_theta_r)
    # print(sim_x,sim_y,sim_theta)
    #

    car0= simulate_car_model.CarModel(car_x, car_y, car_theta_r)

    #cl, cw, min_r, wl, hx, step
    car0.set_infomation(car_l,car_w,min_turning_radiu,wheel_dis,hou_xuan,step)

    route_x, route_y, route_theta = plan.planning(car_x, car_y, car_theta_r)



    # print('----------------------------')
    # for i in range(len(route_x)):
    #     print(route_x[i], route_y[i], route_theta[i])

    # plt.plot(route_x, route_y)
    # plt.show()

    tracking=Tracking(plan,car0,
                      draw=True
                      )

    tracking.parking_tracking(route_x, route_y, route_theta)

"""
函数名称：ros_tracking
输入参数：
    car_real_position_x,car_real_position_y,car_real_position_theta 车辆坐标
    parking_l  车位长度 根据实际标准填写
    road_w     道路宽度
    road_l     道路长度
    car_l        车辆长度
    car_w       车辆宽度
    min_turning_radiu  规划所要求的最小转弯半径
    wheel_dis     车辆轴距
    step     运动步长(常用于模拟时间长度)
    hou_xuan    车辆后悬长度
    real_parking_left_head 车位左前角
    real_parking_right_head 车位右前角
    
输出结果：
   无

函数功能：
    计算一条从给定车辆坐标的位置到车位内部的路线，并调用跟踪方法让车辆停进车位内部
    
函数流程：
     1.将所有环境变量的参数打包进env（环境）类中
     2.将环境类代入规划模块进行环境初始化
     3.调用规划模块的planning方法并获取一条到车位的路径
     4.将车辆数据代入车辆模型并初始化
     5.将规划模块与车辆模型输入跟踪模块并初始化
     6.调用跟踪模块的parking_tracking方法进行ros泊车
"""

def ros_tracking(car_x, car_y, car_theta_r,
                 parking_l, road_w, road_l, car_l, car_w, min_turning_radiu, wheel_dis, step, hou_xuan
                 , real_parking_left_head, real_parking_right_head):

    from car_parking.car_model import ros_car_model
    import carla_info_setting

    #====================================================================================
    # 算法与ros的交互模块
    # 规划，跟踪算法需要　Carla_Info_Setting　往参数空间设置的　车辆位置("/parking_planning/position_x")等信息
    # 跟踪算法计算出的 速度，角度控制信息　会写进参数空间
    #而　Carla_Info_Setting　会读取参数空间的　速度，角度控制信息　并发送对应的ros　ackerman 命令来控制carla的车辆
    #Carla_Info_Setting的线程只会在　"/parking_planning/parking_end"　＝１的情况下运行
    # 线程在参数为0的时候会停止运行
    rospy.set_param("/parking_planning/parking_end", 1)
    time.sleep(0.1)
    info=carla_info_setting.InfoSet()
    info.listener()
    # t = threading.Thread(target=info.listener, args=())
    # t.daemon = True
    # t.start()



    #=====================================================================================
    #衍生的环境信息数据

    # 对于垂直停车，车位宽度由车位前侧左右两点决定，不需要输入
    parking_w = math.sqrt(math.pow(real_parking_left_head[0] - real_parking_right_head[0], 2) + math.pow(
        real_parking_left_head[1] - real_parking_right_head[1], 2))

    #=====================================================================================
    #将信息集中到环境类与车辆类里

    #设置环境
    env0 = Env()
    env0.set_env_info(parking_l, parking_w, road_w, road_l, real_parking_left_head, real_parking_right_head)
    env0.set_car_info(car_l, car_w, min_turning_radiu, wheel_dis, hou_xuan, step)


    #设置车辆模型
    car0= ros_car_model.CarModel(car_x, car_y, car_theta_r)
    car0.set_infomation(car_l,car_w,min_turning_radiu,wheel_dis,hou_xuan,step)

    #=====================================================================================
    #规划设置

    #设置规划方法
    plan = Planning(env0)

    #开始规划
    route_x, route_y, route_theta = plan.planning(car_x, car_y, car_theta_r)
    if route_x==None:
        print("规划路径失败，请移动车辆")
        return
    #打印路径
    print('----------------------------')
    for i in range(len(route_x)):
        print(route_x[i], route_y[i], route_theta[i])

    print('----------------------------')

    #=====================================================================================
    #跟踪设置

    #设置跟踪方法
    # 输入参数有规划方法，还有车辆模型
    # draw的参数是用来画图看俯视视角下，规划算法对车辆的控制的图
    tracking=Tracking(plan,car0,
                      draw=True
                      )
    tracking.tar_speed=2

    #开始跟踪
    tracking.parking_tracking(route_x, route_y, route_theta)




if __name__ == "__main__":#参数来源
    #参数来源可以选择ros 或者其他，其他的时候就自己规定车位坐标与车辆坐标
    # car_position_from="11"
    car_position_from="ros"

    #===============================================================================
    #参数获取

    # 获取车辆位置的方法
    # 当选择ros时，可以用来验证当前车辆所在位置是否能有一条停进车位的路线
    # 不选择ros可以用来测试算法本身有没有问题
    if car_position_from=="ros":
        #从ros中读取车辆坐标
        rospy.set_param("/parking_planning/parking_end", 1)
        info=carla_info_setting.InfoSet()
        car_real_position_x,car_real_position_y,car_real_position_theta =info.get_car_position()
        rospy.set_param("/parking_planning/parking_end", 0)
    else:

        #车辆虚拟坐标
        car_real_position_x =-37.131065
        car_real_position_y =-14.28638
        car_real_position_theta =-1.356844



    # # 车位实际坐标,（前是指靠近车道的那边，左右是指站在车位里看向车道的方向的左右）车位左前角，车位右前角
    # real_parking_left_head = [-42.3, -3.1]
    # real_parking_right_head = [-42.3, -6.1]

    # 新车位车位实际坐标,（前是指靠近车道的那边，左右是指站在车位里看向车道的方向的左右）车位左前角，车位右前角
    real_parking_left_head = [16.6 ,28.4]
    real_parking_right_head = [16.6,26.1]

    # 车位数据，
    parking_l = 5  #车位长度 根据实际标准填写
    road_w = 5.8  #道路宽度
    road_l = 3 * parking_l  #道路长度

    # # 特斯拉 model3 长度4.7，后悬长度1.2
    # # 进行规划所需要的车辆数据
    # car_l = 4.7  # 车辆长度
    # car_w = 1.7  # 车辆宽度
    # min_turning_radiu = 10  # 规划所要求的最小转弯半径
    # wheel_dis = 3.3  # 车辆轴距
    # hou_xuan = 1.2  # 车辆后悬长度

    # 奥迪 a2 数据
    # 进行规划所需要的车辆数据
    car_l = 3.826  # 车辆长度
    car_w = 1.673  # 车辆宽度
    min_turning_radiu = 13  # 规划所要求的最小转弯半径
    wheel_dis = 2.4  # 车辆轴距
    hou_xuan = 0.7  # 车辆后悬长度


    step = 0.3  # 运动步长



    #===============================================================================
    #代码执行部分

    ros_tracking(car_real_position_x, car_real_position_y, car_real_position_theta,
                 parking_l, road_w, road_l, car_l, car_w, min_turning_radiu, wheel_dis, step, hou_xuan
                 , real_parking_left_head, real_parking_right_head)
    # __main_test_simulate()
