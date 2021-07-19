class StrategySpeedDownAndStay():
    def __init__(self, tar_speed, change_time, alive_time):
        self.job_list=[]
        self.end_time=alive_time
        self.change_time= change_time
        self.tar_speed=tar_speed
        self.v=0
        self.w=0

    def get_control(self,time,obs):
        if time>self.change_time:
            self.v=self.tar_speed
        else:
            self.v=self.tar_speed-10

        return self.v,self.w