import copy
import math

import numpy as np
import gym
import highway_env
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from car_orca.pyorca_master.pyorca import Agent, get_avoidance_velocity, orca, normalized, perp,Line
from  car_orca.pyorca_master import  pyorca
from controller import pid_lateral_controller_angle
# from controller import pid_longitudinal_controller

class HighWayOrca():
    def __init__(self,seed=0,method='orca'):
        self.car_steer_limit=math.pi/3
        self.env = gym.make("highway-v0")
        # self.long_pid=pid_longitudinal_controller.PIDLongitudinalController( K_P=1.0, K_D=0.0, K_I=0.0)

        self.fresh_speed=False
        self.env.seed(seed)
        self.done = False
        self.acc = 0.5
        self.tau = 2
        self.dt=0.05
        self.prev = 20
        self.lane=1
        self.lane_length=4

        # 速度P控制器系数 1
        self.Speed_Kp = 0.6
        self.car_radiu=3.2
        self.method=method
        self.edge_remain=0.3
        # config = {
        #     "lanes_count": 3,
        #     "ego_spacing": 0,
        #     'vehicles_count': 15,
        #     'simulation_frequency': 1 / self.dt,  # 20
        #     'vehicles_density': 1.5,
        #     "policy_frequency": 10,  # 10
        #     "duration": 200,
        #     "observation": {
        #         "type": "Kinematics",
        #         "vehicles_count": 6,
        #         "features": ["presence", "x", "y", "vx", "vy", "cos_h", "sin_h"],
        #         "absolute": True,
        #         "normalize": False,
        #         "order": "sorted"
        #     },
        #     "action": {
        #         "type": "ContinuousAction"
        #     }
        # }
        self.config = {
            "lanes_count": 3,
            "ego_spacing": 1,
            'vehicles_count': 15,
            'simulation_frequency': 1 / self.dt,  # 20
            'vehicles_density': 1.5,
            "policy_frequency": 10,  # 10
            "duration": 300,
            "observation": {
                "type": "Kinematics",
                "vehicles_count": 6,
                "features": ["presence", "x", "y", "vx", "vy", "cos_h", "sin_h"],
                "absolute": True,
                "normalize": False,
                "order": "sorted"
            },
            "action": {
                "type": "ContinuousAction",
                "STEERING_RANGE": (-np.pi / 3, np.pi / 3)
            }
        }
        self.env.configure(self.config)
        self.env.reset()

        self.later_pid=pid_lateral_controller_angle.PIDLateralController(L=2.5, dt=self.dt, car_steer_limit=self.car_steer_limit, K_P=4, K_D=0.2, K_I=0)



    #速度控制器
    def __PControl(self, target, current):
        return self.Speed_Kp * (target - current)

    def set_pref_v(self,current_x,current_y,goal_x,goal_y,v):
        theta = math.atan2(goal_y - current_y, goal_x - current_x)
        return  [math.cos(theta)*v,  math.sin(theta)*v]

    def change_vxvy_to_action(self,agent:Agent,control_v):
        l=control_v[0]
        # print(agent.position[0],agent.position[1],agent.theta*180/math.pi)
        control_theta=math.atan2(control_v[1],control_v[0])
        goal_x=math.cos(control_theta)*l
        goal_y=math.sin(control_theta)*l

        now_v=agent.velocity[0]

        steer=self.later_pid.run_step(0, 0, agent.theta, goal_x, goal_y, control_theta, now_v)

        ai= self.__PControl( l,now_v)

        # if steer>math.pi/2:
        #     steer=math.pi/2
        # if steer < -math.pi / 2:
        #     steer = -math.pi / 2

        action=[ai,-steer]

        limit_control=7
        if control_v[0]<agent.velocity[0]-limit_control:
            print('emergy speed down')
            action[0]=-1
        if control_v[0]>agent.velocity[0]+limit_control:
            print('emergy speed up')
            action[0]=1
        # print('pose ', agent.position, agent.theta * 180 / math.pi, ' now_speed ', agent.velocity, 'pre-v',
        #       agent.pref_velocity, 'orca-v', control_v, 'action ', action)

        return action
        # return [0.5,0]

    def get_agent_from_obs(self,obj):
        position=(obj[1], obj[2])
        velocity = (obj[3], obj[4])

        ag=Agent(position,velocity, self.car_radiu,  self.acc, (obj[3], obj[4]),theta=math.atan2(obj[6],obj[5]))
        # if math.atan2(obj[6],obj[5])!=0:
        #     print('obj ', obj, '  theta ', math.atan2(obj[6], obj[5]))
        vx,vy=pyorca.get_vxvy_from_agent(ag)
        ag.velocity=np.array((vx,vy))
        return ag

    def run(self,switch=9999999):
        count=0
        action = (0, 0)
        crash=False
        accdent_list=[]
        accdent_len=8
        while not self.done:
            count+=1
            # action =env.action_space.sample()
            obs, reward, self.done, info = self.env.step(action)
            # print('---------------------------------------------')
            # print('run count ',count)






            # print('action ',action, type(action))
            # print('obs ',obs)
            if self.done:
                if info["crashed"]==True:
                    accdent_numb=0
                    # print(obs)
                    for i in range(1,len(obs)):
                        obj=obs[i]
                        # if obj[3]!=0:
                        #     continue

                        if math.hypot(obj[1]-obs[0][1],obj[2]-obs[0][2])<2*accdent_len:
                            accdent_numb+=1
                    # print('accdent_numb ',accdent_numb)
                    if accdent_numb>1:
                        crash=True
                    # print("crash")
                else:
                    a=0
                    # print("done")
                break

            agents = []
            new_speed=0
            for obj in obs:
                pd=False
                new_speed+=obj[3]
                for i in obj:
                    if i !=0:
                        pd=True
                if pd:
                    agents.append(self.get_agent_from_obs(obj))
                    # print('pre ',obj[0],' x ',obj[1],' y ',obj[2],' vx ',obj[3],' vy ',obj[4],' cosh ',obj[5],' sinh ',obj[6] )
            # print('info ',info)
            if self.fresh_speed:
                new_speed=new_speed/len(obs)
                if new_speed!=0:
                    int(new_speed)
                    self.prev=new_speed+5

            # 设置目标速度，换道决策
            agents[0].pref_velocity = self.set_pref_v(agents[0].position[0], agents[0].position[1],
                                                 agents[0].position[0] + 80, self.lane*self.lane_length, self.prev)
            # print('pose ',agents[0].position,' theta ',theta*180/math.pi,math.sin(theta))

            if count<=switch:
                # 计算orca避障速度
                new_vels, all_line = orca(agents[0], agents[1:], self.tau, self.dt
                                          , limit=[-2+agents[0].radius/2+self.edge_remain,14-agents[0].radius/2-self.edge_remain]
                                          , method=self.method)
            else:
                # 计算orca避障速度
                # print('orca switch')
                new_vels, all_line = orca(agents[0], agents[1:], self.tau, self.dt
                                          , limit=[-2+agents[0].radius/2+self.edge_remain,14-agents[0].radius/2-self.edge_remain]
                                          , method="orca")

            # 将速度转换为动作指令
            action = self.change_vxvy_to_action(agents[0], new_vels)
            new_v=new_vels


            self.env.render()


            # input()
            # if count==197:
            #     input()
            if count>=0:
                # for i in range(1,len(agents)):
                #     self.draw_orca_collider(agents[0],agents[i],self.tau, self.dt,limit=[-2+agents[0].radius/2+self.edge_remain,14-agents[0].radius/2-self.edge_remain])

                self.draw(agents[0], agents[1:],all_line,new_v)
                # self.draw_speed_reward(new_v,agents[0],all_line)
        return crash,count
    def draw_orca_collider(self,agent, collider, t, dt,limit):
        import draw_picture

        x = -(agent.position - collider.position)
        r = agent.radius + collider.radius

        x_len_sq = pyorca.norm_sq(x)

        if x_len_sq < r * r:
            return None,None
        print('agent ',agent.position,agent.velocity,' collider',collider.position,collider.velocity)
        collider_aviliable_speed_set=pyorca.get_car_aciliable_speed(collider,t,limit)
        unino_agent_collide_speed_set=pyorca.get_agent_collide_set(agent,collider,t)
        agent_collide_set=pyorca.Minkowski.Minkowski_sum(collider_aviliable_speed_set,unino_agent_collide_speed_set)
        u,n=pyorca.get_dv_n_from_tubianxing(agent.velocity,agent_collide_set)

        print(collider_aviliable_speed_set)
        print(unino_agent_collide_speed_set)


        plt.figure(num='car_orca')
        plt.clf()
        # 画图
        x_major_locator = MultipleLocator(10)
        y_major_locator = MultipleLocator(10)
        ax = plt.gca()# ax为两条坐标轴的实例
        ax.xaxis.set_major_locator(x_major_locator)
        # 把x轴的主刻度设置为1的倍数
        ax.yaxis.set_major_locator(y_major_locator)


        for i in range(len(collider_aviliable_speed_set)):
            collider_aviliable_speed_set[i]+=x

        plt.scatter(0,0,color='red')
        plt.scatter(x[0],x[1],color='blue')

        plt.plot([0,agent.velocity[0]],[0,agent.velocity[1]],color='black')
        plt.plot([agent.velocity[0],agent.velocity[0]+u[0]],[agent.velocity[1],agent.velocity[1]+u[1]],color='red')

        draw_picture.draw_tubianxing(collider_aviliable_speed_set,color='blue')
        draw_picture.draw_tubianxing(unino_agent_collide_speed_set,color='red')
        draw_picture.draw_tubianxing(agent_collide_set,color='black')
        plt.show()


        return u,n

    def draw(self, my_agent:Agent, agents, lines,v_opt):
        #设置显示范围
        show_range=30
        ylimit=[-5,18]
        show_rate=12

        plt.figure(num='route', figsize=(show_range*6/show_rate,(ylimit[1]-ylimit[0])/show_rate) )
        plt.clf()
        # 画图
        x_major_locator = MultipleLocator(10)
        y_major_locator = MultipleLocator(10)
        ax = plt.gca()# ax为两条坐标轴的实例
        ax.xaxis.set_major_locator(x_major_locator)
        # 把x轴的主刻度设置为1的倍数
        ax.yaxis.set_major_locator(y_major_locator)
        plt.xlim(my_agent.position[0] - show_range * 3, my_agent.position[0] + show_range * 3)
        plt.ylim(ylimit)
        plt.gca().invert_yaxis()



        #绘制车道线
        for i in range(5):
            plt.plot([my_agent.position[0] - show_range * 3, my_agent.position[0] + show_range * 3], [i * 4 - 2, i * 4 - 2], color='grey')

        #绘制障碍物
        for ag in agents:
            self.draw_circle(ag)

        #绘制半平面
        for line in lines:
            # print('before ',line)
            self.draw_half_line(my_agent,line)
        # print('velo ',my_agent.velocity)
        # 绘制控制车辆
        self.draw_circle(my_agent, colr='black')

        # 绘制控制车辆各种速度
        # plt.plot([my_agent.position[0],my_agent.position[0]+my_agent.pref_velocity[0]],[my_agent.position[1],my_agent.position[1]+my_agent.pref_velocity[1]], color='red')
        plt.plot([my_agent.position[0], my_agent.position[0] + v_opt[0]],
                 [my_agent.position[1], my_agent.position[1] + v_opt[1]], color='blue')

        # print(my_agent.velocity[1],np.arctan(1 / 2 * np.tan(my_agent.velocity[1])))
        # beta = np.arctan(1 / 2 * np.tan(my_agent.velocity[1]))
        # print('pre pose ', my_agent.position[0] + my_agent.velocity[0] * np.cos(my_agent.theta + beta) * 0.02,
        #       my_agent.position[1] + my_agent.velocity[1] * np.cos(my_agent.theta + beta) * 0.02)

        # plt.show()
        plt.pause(0.01)
        # input()

    def draw_speed_reward(self,orca_v,agent,lines):
        from mpl_toolkits.mplot3d import Axes3D
        import matplotlib.pyplot as plt
        from matplotlib import cm
        from matplotlib.ticker import LinearLocator, FormatStrFormatter
        import numpy as np

        x_range=30
        x=[]
        y=[]
        z=[]
        max_reward=10
        for i in range(int(orca_v[0])-x_range,int(orca_v[0])+x_range):
            x_arr=[]
            y_arr=[]
            z_arr=[]
            for j in range(-30,150,1):
                # k=i+j
                k=-pyorca.get_speed_reward(lines, agent,[i,j/10],self.tau,self.dt)
                if k<-max_reward:
                    k=-max_reward
                x_arr.append(i)
                y_arr.append(j)
                z_arr.append(k)
            x.append(x_arr)
            y.append(y_arr)
            z.append(z_arr)

        z=np.array(z)
        x=np.array(x)
        y=np.array(y)



        fig = plt.figure()
        ax = fig.gca(projection='3d')
        surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='rainbow')

        ax.set_xlabel('road')
        ax.set_ylabel('lanes')
        ax.set_zlabel('reward')
        ax.set_zlim(-10, 2)
        ax.set_ylim(-3, 15)
        ax.zaxis.set_major_locator(LinearLocator(10))

        fig.colorbar(surf, shrink=1, aspect=10)


        # Add a color bar which maps values to colors.

        plt.show()

        return

    def draw_circle(self, agent, colr="r"):
        r=agent.radius
        x=agent.position[0]
        y=agent.position[1]
        # 点的横坐标为a
        a = np.arange(x - r, x + r, 0.001)
        # 点的纵坐标为b
        b = np.sqrt(np.power(r, 2) - np.power((a - x), 2))
        plt.plot(a, b+ y, color=colr, linestyle='-')
        plt.plot(a, -b+ y, color=colr, linestyle='-')
        plt.scatter(x, y, c='b', marker='o')
        plt.plot([x,x+agent.velocity[0]/5],[y,y+agent.velocity[1]/5], color=colr, linestyle='-')

    def draw_half_line(self,agent:Agent,line:Line):
        len=50
        init_pose=agent.position+line.point
        # print('after ',init_pose,agent.position,line.point)
        plt.plot([init_pose[0],init_pose[0]+line.direction[0]],[init_pose[1],init_pose[1]+line.direction[1]],color='green')
        pre=perp(line.direction)
        plt.plot([init_pose[0],init_pose[0]+pre[0]*len],[init_pose[1],init_pose[1]+pre[1]*len],color='green')
        plt.plot([init_pose[0],init_pose[0]-pre[0]*len],[init_pose[1],init_pose[1]-pre[1]*len],color='green')

        # plt.scatter(init_pose[0],init_pose[1],color='green')

def main():
    ans=[]
    #194,273,349
    coll =  [92, 94, 99, 109, 125, 138, 183, 184, 194, 205, 221, 231, 251, 273, 312, 349, 350, 357,
             360, 363, 373, 375, 377, 382, 383, 384, 386, 387, 389, 390, 395, 396, 399, 401, 403, 406, 407, 409, 410, 418,
             422, 423, 424, 426, 432, 434, 436, 441, 443, 444, 445, 461, 463, 465, 466, 467, 469, 471, 475, 476, 478, 483,
             487, 496, 497, 501, 504, 505, 507, 509, 511, 515, 517, 519, 522, 530, 536, 537, 544, 545, 548, 553, 554, 561,
             562, 569, 574, 577, 578, 579, 584, 589, 591, 596, 598, 600, 602, 605, 606, 611, 615, 617, 618, 625, 631, 632,
             633, 635, 637, 640, 649, 651, 652, 653, 654, 655, 657, 658, 661, 669, 671, 672, 673, 674, 675, 676, 679, 680,
             683, 684, 685, 686, 687, 697, 698, 700, 705, 706, 710, 711, 712, 717, 724, 735, 741, 752, 754, 756, 758, 770,
             774, 778, 780, 782, 787, 795, 800, 801, 803, 806, 811, 818, 823, 827, 837, 838, 841, 843, 844, 848, 854, 855,
             858, 859, 860, 862, 863, 866, 867, 877, 878, 881, 883, 884, 885, 887, 892, 893, 896, 898, 902, 904, 905, 907,
             908, 914, 917, 918, 924, 928, 930, 936, 937, 939, 945, 953, 955, 959, 963, 964, 965, 966, 970, 972, 977, 978,
             981, 986, 988, 992, 994, 995, 997, 998, 1001, 1003, 1010, 1011, 1013, 1014, 1018, 1019, 1029, 1030, 1034, 1036,
             1037, 1038, 1040, 1041, 1042, 1051, 1052, 1053, 1055, 1058, 1059, 1064]
    new_coll=[]
    coll=[349]
    print(len(coll))
    for seed in coll:
        new_highway_orca=HighWayOrca(seed,'avo')
        tmp_crash,tmp_count=new_highway_orca.run()
        if tmp_crash==True:
            new_highway_orca = HighWayOrca(seed, 'avo')
            if tmp_count > 80:
                orca_crash, orca_count = new_highway_orca.run(switch=tmp_count - 80)
            else:
                orca_crash, orca_count = new_highway_orca.run(switch=0)
            if orca_crash==False:
                new_coll.append(seed)
                print(new_coll)

    # for seed in range(0,10000):
    #     print('=====================================================')
    #     print('now seed is ',seed)
    #     print('now list is = ',ans)
    #     new_highway_orca=HighWayOrca(seed,'avo')
    #     tmp_crash,tmp_count=new_highway_orca.run()
    #     if tmp_crash==False:
    #         continue
    #     new_highway_orca=HighWayOrca(seed,'avo')
    #     if tmp_count>80:
    #         orca_crash,orca_count=new_highway_orca.run(switch=tmp_count-80)
    #     else:
    #         orca_crash,orca_count=new_highway_orca.run(switch=0)
    #     if orca_crash==True:
    #         continue
    #     ans.append(seed)
    #     print('update list is = ',ans)




if __name__ == '__main__':

    main()