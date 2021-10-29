import math

from env.highway_sim_env import HighwaySimulation
from orca_src.pyorca import orca

from orca_src.car_orca import Orca


class HighWayOrca(Orca):
    def __init__(self,sim_env=None,method='avo'):
        Orca.__init__(self,method='avo')
        self.fresh_speed=False
        self.env=sim_env
        self.done = False

    # et orca_src action from orca_src
    def get_orca_action(self, obs, method=None):

        # 将obs的数据格式转换成orca使用的agents类型
        agents = []
        for obj in obs:
            pd = False
            for i in obj:
                if i != 0:
                    pd = True
            if pd:
                agents.append(self.get_agent_from_obj(obj))
                # print('pre ',obj[0],' x ',obj[1],' y ',obj[2],' vx ',obj[3],' vy ',obj[4],' cosh ',obj[5],' sinh ',obj[6] )

        # 设置目标速度，换道决策
        agents[0].pref_velocity = self.set_pref_v(agents[0].position[0], agents[0].position[1],
                                                              agents[0].position[0] + 80,
                                                              self.lane * self.lane_length,
                                                              self.prev)
        # print('pose ',agents[0].position,' theta ',theta*180/math.pi,math.sin(theta))

        # 计算orca避障速度
        if method == None:
            new_vels, all_line = orca(agents[0], agents[1:], self.env.tau, self.env.dt,
                                      limit=[-2 + agents[0].radius / 2 + self.edge_remain,
                                             14 - agents[0].radius / 2 - self.edge_remain],
                                      method=self.method)
        else:
            new_vels, all_line = orca(agents[0], agents[1:], self.env.tau, self.env.dt,
                                      limit=[-2 + agents[0].radius / 2 + self.edge_remain,
                                             14 - agents[0].radius / 2 - self.edge_remain], method=method)

        # 将速度转换为动作指令
        action = self.change_vxvy_to_action(agents[0], new_vels)

        return action, new_vels

    def run(self,switch=9999999):
        count=0
        action = (0, 0)
        crash=False


        while not self.done:
            count+=1
            # action =env.action_space.sample()
            obs, reward, self.done, info = self.env.step(action)

            #  for-path
            carlist = []
            veh = self.env.road.vehicles
            if veh:
                for v in veh:
                    Ind = count
                    carlist.append([Ind, v.position[0], v.position[1], v.heading])

            if self.done:
                if info["crashed"]==True:
                    crash=True
                    # print("crash")
                break

            agents = self.get_agents_from_obs(obs)


            # 设置目标速度，换道决策
            agents[0].pref_velocity = self.set_pref_v(agents[0].position[0], agents[0].position[1],
                                                 agents[0].position[0] + 80, self.lane*self.lane_length, self.prev)


            action=self.get_action_from_agents(agents,method0=self.method)


            if self.fresh_speed:
                new_speed=new_speed/len(obs)
                if new_speed!=0:
                    int(new_speed)
                    self.prev=new_speed+5


            if count<=switch:
                # 计算orca避障速度
                new_vels, all_line = orca(agents[0], agents[1:], self.tau, self.dt
                                          , limit=[-2+agents[0].radius/2+self.edge_remain,14-agents[0].radius/2-self.edge_remain]
                                          , method=self.method)
            else:
                # 计算orca避障速度
                # print('orca_src switch')
                new_vels, all_line = orca(agents[0], agents[1:], self.tau, self.dt
                                          , limit=[-2+agents[0].radius/2+self.edge_remain,14-agents[0].radius/2-self.edge_remain]
                                          , method="orca_src")

            # 将速度转换为动作指令
            new_vels[0] = new_vels[0]+1
            new_vels[1] = new_vels[1] + 0.3
            action = self.change_vxvy_to_action(agents[0], new_vels)




            new_v=new_vels


            self.env.render()


            # input()
            # if count==197:
            #     input()
            if count>=0:
                print('[')
                for obj in obs:
                    print('[',count,',%.2f' %obj[1],',%.2f' %obj[2],',%.2f' %(math.atan2(obj[6],obj[5])),'],')
                print('],')
                # for i in range(1,len(agents)):
                #     self.draw_orca_collider(agents[0],agents[i],self.tau, self.dt,limit=[-2+agents[0].radius/2+self.edge_remain,14-agents[0].radius/2-self.edge_remain])

                self.draw(agents[0], agents[1:],all_line,new_v)
                # self.draw_speed_reward(new_v,agents[0],all_line)
        return crash,count


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
    # for seed in coll:
    #     print('=========================')
    #     new_highway_orca=HighWayOrca(seed,'avo')
    #     tmp_crash,tmp_count=new_highway_orca.run()
    #     print('=========================')
    #     if tmp_crash==True:
    #         new_highway_orca = HighWayOrca(seed, 'avo')
    #         if tmp_count > 80:
    #             orca_crash, orca_count = new_highway_orca.run(switch=tmp_count - 80)
    #         else:
    #             orca_crash, orca_count = new_highway_orca.run(switch=0)
    #
    #         if orca_crash==False:
    #             new_coll.append(seed)
    #             print(new_coll)

    for seed in range(0,10000):
        print('=====================================================')
        print('now seed is ',seed)
        print('now list is = ',ans)
        sim_env = HighwaySimulation(seed)

        new_highway_orca=HighWayOrca(sim_env,'avo')
        tmp_crash,tmp_count=new_highway_orca.run()
        if tmp_crash==False:
            continue
        new_highway_orca=HighWayOrca(sim_env,'avo')
        if tmp_count>80:
            orca_crash,orca_count=new_highway_orca.run(switch=tmp_count-80)
        else:
            orca_crash,orca_count=new_highway_orca.run(switch=0)
        if orca_crash==True:
            continue
        ans.append(seed)
        print('update list is = ',ans)




if __name__ == '__main__':

    main()