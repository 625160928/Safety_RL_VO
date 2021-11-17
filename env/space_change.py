import  math
import numpy as np

class SpaceChange():
    def __init__(self,real_point1,real_point2,sim_point1,sim_point2):
        self.real_point1 = real_point1
        self.real_point2=real_point2
        self.sim_point1 = sim_point1
        self.sim_point2 = sim_point2
        # print('r-s')
        self.real_to_sim_theta_r,self.real_to_sim_a,  self.real_to_sim_b=self.change_init(self.real_point1,self.real_point2,self.sim_point1,self.sim_point2)
        # print('rend ',self.real_to_sim_theta_r,self.real_to_sim_a,  self.real_to_sim_b)
        # print('s-r')
        self.sim_to_real_theta_r, self.sim_to_real_a, self.sim_to_real_b = self.change_init(self.sim_point1,self.sim_point2,self.real_point1,self.real_point2)
        # print('send ',self.sim_to_real_theta_r, self.sim_to_real_a, self.sim_to_real_b)

    def real_to_sim(self,x,y,theta):
        x_=(float)(x*math.cos(self.real_to_sim_theta_r)-y*math.sin(self.real_to_sim_theta_r)+self.real_to_sim_a)
        y_=(float)(y*math.cos(self.real_to_sim_theta_r)+x*math.sin(self.real_to_sim_theta_r)+self.real_to_sim_b)
        return x_,y_,theta+self.real_to_sim_theta_r

    def sim_to_real(self,x,y,theta):
        # print('real to sin ',self.sim_to_real_theta_r,math.cos(self.sim_to_real_theta_r),math.sin(self.sim_to_real_theta_r),self.sim_to_real_a,self.sim_to_real_b)
        x_=(float)(x*math.cos(self.sim_to_real_theta_r)-y*math.sin(self.sim_to_real_theta_r)+self.sim_to_real_a)
        y_=(float)(y*math.cos(self.sim_to_real_theta_r)+x*math.sin(self.sim_to_real_theta_r)+self.sim_to_real_b)
        return x_,y_,theta+self.sim_to_real_theta_r

    def change_init(self,rp1,rp2,sp1,sp2):
        a=(rp2[0] - rp1[0])
        b=(sp2[0] - sp1[0])
        c=(sp2[1] - sp1[1])

        m = np.array([[rp1[0], -rp1[1], 1, 0], [rp1[1], rp1[0], 0, 1], [rp2[0], -rp2[1], 1, 0], [rp2[1], rp2[0], 0, 1]])
        n = np.array([sp1[0], sp1[1], sp2[0], sp2[1]])  # 可替换为式子右边的常数

        solution = np.linalg.solve(m, n)
        theta_r = math.atan2(solution[1], solution[0])
        # print(solution)
        # theta_r=math.acos(a/math.sqrt(b*b+c*c))-math.acos(b/math.sqrt(b*b+c*c))
        # r_a=rp1[0]-sp1[0]*math.cos(theta_r)+sp1[1]*math.sin(theta_r)
        # r_b=rp1[1]-sp1[1]*math.cos(theta_r)-sp1[0]*math.sin(theta_r)
        r_a=solution[2]
        r_b = solution[3]
        # print(theta_r*180/math.pi,math.sin(theta_r),math.cos(theta_r),r_a,r_b)


        # print("ans",theta_r,r_a,r_b)


        return theta_r,r_a,r_b

if __name__ == "__main__":
    # rp1=[-31.6,25.5]
    # rp2=[-31.6,28.7]
    rp1=[-42.3,-3.1]
    rp2=[-42.3,-6.1]
    parking_w_tmp=math.sqrt(math.pow(rp1[0]-rp2[0],2)+math.pow(rp1[1]-rp2[1],2))
    sp1=[parking_w_tmp,0]
    sp2=[2*parking_w_tmp,0]
    print('rp1 ',rp1)
    print('sp1 ',sp1)
    print('rp2 ', rp2)
    print('sp2 ', sp2)
    spc=SpaceChange(rp1, rp2, sp1, sp2)
    print('spc!!!!!!!!!!!!!!!!!!!')
    # listener()

    # car_pos=get_pos()
    # ox,oy,z=car_pos.get_car_position()
    # print(ox,oy,z)
    print(spc.real_to_sim(-37.77975082397461,-11.706940650939941,-1.4873837232589722))
    # print(spc.sim_to_real(-37.77975082397461,-11.706940650939941,-1.4873837232589722))
    # print(spc.real_to_sim(-42.29999923706055,-3.0999999046325684,0))

