# Copyright (c) 2013 Mak Nazecic-Andrlon
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Implementation of the 2D ORCA algorithm as described by J. van der Berg,
S. J. Guy, M. Lin and D. Manocha in 'Reciprocal n-body Collision Avoidance'."""

from __future__ import division

import math

import numpy
from numpy import array, sqrt, copysign, dot
from numpy.linalg import det
import numpy as np

from halfplaneintersect import halfplane_optimize, Line, perp

# Method:
# For each robot A and potentially colliding robot B, compute smallest change
# in relative velocity 'u' that avoids collision. Find normal 'n' to VO at that
# point.
# For each such velocity 'u' and normal 'n', find half-plane as defined in (6).
# Intersect half-planes and pick velocity closest to A's preferred velocity.

class Agent(object):
    """A disk-shaped agent."""
    def __init__(self, position, velocity, radius, max_speed, pref_velocity,theta=None):
        super(Agent, self).__init__()
        self.position = array(position)
        self.velocity = array(velocity)
        self.radius = radius
        self.max_speed = max_speed
        self.pref_velocity = array(pref_velocity)
        self.theta=theta

limit_range=[-20,34]

def orca(agent, colliding_agents, t, dt, limit=None):
    """Compute ORCA solution for agent. NOTE: velocity must be _instantly_
    changed on tick *edge*, like first-order integration, otherwise the method
    undercompensates and you will still risk colliding."""
    # print('orca ',colliding_agents)
    if limit is None:
        limit = limit_range
    lines = []
    for collider in colliding_agents:
        # print(collider)
        dv, n = get_avoidance_velocity(agent, collider, t, dt)
        line = Line(agent.velocity + dv / 2, n)
        lines.append(line)
        print('rela ',collider.position,collider.velocity,' -- ',agent.velocity + dv / 2, n)
    lines.append( Line([0,limit[0]-agent.position[1]], [0,1]))
    lines.append( Line([0,limit[1]-agent.position[1]], [0,-1]))
    # print('limit ',limit)
    # print("?? ",[0,limit[0]-agent.position[1]], [0,1])
    # return halfplane_optimize(lines, agent.pref_velocity), lines
    return control_potimize(lines, agent,t,dt), lines

def control_potimize(lines, agent,t,dt):
    derta_acc=0.1
    derta_steer=0.05
    acc_range_number=3
    steer_range_number=3
    contorl_arr=[]
    reward_arr=[]
    for i in range(-acc_range_number,acc_range_number+1):
        for j in range(-steer_range_number,steer_range_number+1):
            control=control_create(agent, i * derta_acc, j * derta_steer)
            # print(control)

            tmp_reward= reward_control(agent,control, t, dt, lines)

            contorl_arr.append(control)
            reward_arr.append(tmp_reward)

    final_control=control_choose(agent,reward_arr,contorl_arr,t)

    print('action ',final_control,' pre ',control_v_create(agent, final_control, t))
    return final_control

def control_choose(agent:Agent,rewards,actions,t):
    min_coll=9999
    for i in range(len(rewards)):
        if rewards[i][0]<min_coll:
            min_coll=rewards[i][0]

    min_reward=99999
    min_action=[]
    for i in range(len(rewards)):
        if rewards[i][0]==min_coll:
            control_v=control_v_create(agent, actions[i], t)
            tmp_reward=rewards[i][1]+math.hypot(agent.pref_velocity[0]-control_v[0],agent.pref_velocity[1]-control_v[1])
            print("action ",rewards[i],actions[i],math.hypot(agent.pref_velocity[0]-control_v[0],agent.pref_velocity[1]-control_v[1]))
            if tmp_reward<min_reward:
                min_reward=tmp_reward
                min_action=actions[i]


    print("control choose")
    return min_action

def control_create(agent, a_speed, a_w):
    return [a_speed, a_w]

# 简单的位置计算
def control_v_create(agent:Agent,control, t):
    theta=agent.theta
    v=agent.velocity[0]+control[0]*t
    v_pose=[v*math.cos(theta),v*math.sin(theta)]
    v_w=[control[1]*t*math.sin(theta),control[1]*t*math.cos(theta)]
    return [v_pose[0]+v_w[0],v_pose[1]+v_w[1]]

def reward_control(agent,control, t, dt, lines,route_anylize=False):

    if route_anylize :
        t_count=int(t/dt)
        reward=[0,0]
        for i in range(t_count):
            control_v = control_v_create(agent,control, i*dt)
            tmp_reward=reward_point_to_lines(control_v,lines)
            reward[0]+=tmp_reward[0]
            reward[1]+=tmp_reward[1]
        reward[0]=reward[0]/t_count
        reward[1] = reward[1] / t_count
    else:
        control_v = control_v_create(agent,control, t)
        reward=reward_point_to_lines(control_v,lines)
    return reward

def reward_point_to_lines(point,lines):
    reward=0
    count=0
    for line in lines:
        re=reward_point_to_line(point,line)
        if re !=0:
            count+=1
            reward+=re
    return [count,reward]

def reward_point_to_line(point,line):
    x=np.array(point-line.point)
    y=np.array(line.direction)
    Lx = np.sqrt(x.dot(x))
    Ly = np.sqrt(y.dot(y))
    v3=np.array(x-y)
    Lc=np.sqrt(v3.dot(v3))
    cos_angle = (Lx*Lx+Ly*Ly-Lc*Lc)/(2*Ly*Lx)
    # print(cos_angle*180/math.pi)
    # theta = math.acos(cos_angle)
    if cos_angle>1:
        cos_angle=1
    if cos_angle<-1:
        cos_angle=-1
    theta=math.atan2(math.sqrt(1-cos_angle*cos_angle ),cos_angle)
    # print(theta/math.pi*180)
    if theta<=math.pi/2:
        return 0
    else:
        if line.direction[0]!=0:
            k=line.direction[1]/line.direction[0]
            distant=abs(k*x[0]-x[1])/math.sqrt(k*k+1)
        else:
            distant=abs(x[0])
        return distant

def get_avoidance_velocity(agent, collider, t, dt):
    """Get the smallest relative change in velocity between agent and collider
    that will get them onto the boundary of each other's velocity obstacle
    (VO), and thus avert collision."""

    # This is a summary of the explanation from the AVO paper.
    #
    # The set of all relative velocities that will cause a collision within
    # time tau is called the velocity obstacle (VO). If the relative velocity
    # is outside of the VO, no collision will happen for at least tau time.
    #
    # The VO for two moving disks is a circularly truncated triangle
    # (spherically truncated cone in 3D), with an imaginary apex at the
    # origin. It can be described by a union of disks:
    #
    # Define an open disk centered at p with radius r:
    # D(p, r) := {q | ||q - p|| < r}        (1)
    #
    # Two disks will collide at time t iff ||x + vt|| < r, where x is the
    # displacement, v is the relative velocity, and r is the sum of their
    # radii.
    #
    # Divide by t:  ||x/t + v|| < r/t,
    # Rearrange: ||v - (-x/t)|| < r/t.
    #
    # By (1), this is a disk D(-x/t, r/t), and it is the set of all velocities
    # that will cause a collision at time t.
    #
    # We can now define the VO for time tau as the union of all such disks
    # D(-x/t, r/t) for 0 < t <= tau.
    #
    # Note that the displacement and radius scale _inversely_ proportionally
    # to t, generating a line of disks of increasing radius starting at -x/t.
    # This is what gives the VO its cone shape. The _closest_ velocity disk is
    # at D(-x/tau, r/tau), and this truncates the VO.

    x = -(agent.position - collider.position)
    v = agent.velocity - collider.velocity
    r = agent.radius + collider.radius

    x_len_sq = norm_sq(x)

    if x_len_sq >= r * r:
        # We need to decide whether to project onto the disk truncating the VO
        # or onto the sides.
        #
        # The center of the truncating disk doesn't mark the line between
        # projecting onto the sides or the disk, since the sides are not
        # parallel to the displacement. We need to bring it a bit closer. How
        # much closer can be worked out by similar triangles. It works out
        # that the new point is at x/t cos(theta)^2, where theta is the angle
        # of the aperture (so sin^2(theta) = (r/||x||)^2).
        adjusted_center = x/t * (1 - (r*r)/x_len_sq)

        if dot(v - adjusted_center, adjusted_center) < 0:
            # v lies in the front part of the cone
            # print("front")
            # print("front", adjusted_center, x_len_sq, r, x, t)
            w = v - x/t
            u = normalized(w) * r/t - w
            n = normalized(w)
        else: # v lies in the rest of the cone
            # print("sides")
            # Rotate x in the direction of v, to make it a side of the cone.
            # Then project v onto that, and calculate the difference.
            leg_len = sqrt(x_len_sq - r*r)
            # The sign of the sine determines which side to project on.
            sine = copysign(r, det((v, x)))
            rot = array(
                ((leg_len, sine),
                (-sine, leg_len)))
            rotated_x = rot.dot(x) / x_len_sq
            n = perp(rotated_x)
            if sine < 0:
                # Need to flip the direction of the line to make the
                # half-plane point out of the cone.
                n = -n
            # print("rotated_x=%s" % rotated_x)
            u = rotated_x * dot(v, rotated_x) - v
            # print("u=%s" % u)
    else:
        # We're already intersecting. Pick the closest velocity to our
        # velocity that will get us out of the collision within the next
        # timestep.
        # print("intersecting")
        w = v - x/dt
        u = normalized(w) * r/dt - w
        n = normalized(w)
    return u, n

def norm_sq(x):
    return dot(x, x)

def normalized(x):
    l = norm_sq(x)
    assert l > 0, (x, l)
    return x / sqrt(l)

def dist_sq(a, b):
    return norm_sq(b - a)


if __name__ == '__main__':
    # print(reward_point_to_line(point=[0,10],line=Line([0,0],[-1,-1])))
    control_potimize(0,0,0,0)