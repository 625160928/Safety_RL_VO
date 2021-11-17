import numpy as np
import math

from env import space_change

class GridMap():
    def __init__(self):
        self.x_length=10
        self.y_length=10
        self.xy_resolusion=0.1
        self.theta_resolusion=10
        self.map=self.gengerate_map()
        self.zero_x=0
        self.zero_y=0
        self.zero_theta=0 #rad

        self.space_change=space_change.SpaceChange([self.zero_x,self.zero_y],[self.zero_x+math.cos(self.zero_theta),self.zero_y+math.sin(self.zero_theta)],[0,0],[1,0])

    def gengerate_map(self):
        return np.zeros((int(self.x_length/self.xy_resolusion),int(self.y_length/self.xy_resolusion)), dtype=int)

    def reset_space_change(self):
        self.space_change = space_change.SpaceChange([self.zero_x, self.zero_y],
                                                     [self.zero_x + math.cos(self.zero_theta),
                                                      self.zero_y + math.sin(self.zero_theta)], [0, 0], [1, 0])

    #return false mean have no collision
    def check_collision(self,position_x,position_y,position_theta):
        map_position_x,map_position_y,map_position_theta=self.space_change.real_to_sim(position_x,position_y,position_theta)
        if map_position_x<0 or map_position_y<0 or map_position_x>self.x_length or map_position_y>self.y_length:
            return True
        if self.map[int(map_position_x/self.xy_resolusion)][int(map_position_y/self.xy_resolusion)]==1:
            return True
        return False






if __name__ == '__main__':
    grid_map=GridMap()
    print(grid_map.check_collision(5,0,0))