import matplotlib.pyplot as plt

import math
import time
from orca_src import class_line


def SubPoint(point1, point2):
    x = point1[0] - point2[0]
    y = point1[1] - point2[1]
    return (x, y)

def CrossProduct(point1, point2):
    return point1[0] * point2[1] - point1[1] * point2[0]

#Pan duan shi fou shun shi zhen
def IsDisOrder(rec):
    v1=[rec[1][0]-rec[0][0],rec[1][1]-rec[0][1]]
    v2=[rec[2][0]-rec[1][0],rec[2][1]-rec[1][1]]
    # print('is order ',CrossProduct(v1,v2))
    if CrossProduct(v1,v2)>0:
        return True
    return False

def IsPointInConvexPolygon(pointlst, targetp):
    length = len(pointlst)
    if length == 0:
        ValueError("pointlst is wrong")
    nCurCrossProduct = 0.0
    nLastValue = 0.0
    for i in range(length):
        indbegin = i
        indend = i + 1
        if indend == length:
            indend = 0
        vU = SubPoint(targetp, pointlst[indbegin])
        vV = SubPoint(targetp, pointlst[indend])
        nCurCrossProduct = CrossProduct(vU, vV)
        if (i > 0  and (nCurCrossProduct * nLastValue <= 0)):
            return False
        nLastValue = nCurCrossProduct
    return True

def get_min_point_in_tubianxing(point,point_list):
    min_dist=None
    min_point=None
    mins_seg=None
    for i in range(len(point_list)):
        if i==0:
            line=[point_list[0],point_list[len(point_list)-1]]
            projx,projy= class_line.proj_from_point_to_segment(point, line)
        else:
            line=[point_list[i],point_list[i-1]]
            projx,projy= class_line.proj_from_point_to_segment(point, line)
        tmp_dist=math.hypot(projx-point[0],projy-point[1])
        if min_dist==None:
            min_dist=tmp_dist
            min_point=[projx,projy]
            mins_seg=line
        elif tmp_dist<min_dist:
            min_dist=tmp_dist
            min_point=[projx,projy]
            mins_seg=line

    #     print(i,tmp_dist,projx,projy)
    # print(min_dist,min_point)
    return min_point,mins_seg



def main():
    c=[[-7, 10], [-9, 8], [-7, 5], [-4, 3], [-1, 3], [1, 6], [1, 7], [0.5, 8], [-1.0, 10]]
    check_point=[3,10]

    get_min_point_in_tubianxing(check_point,c)

    print(IsPointInConvexPolygon(c,check_point))

    plt.scatter(check_point[0],check_point[1])
    plt.xlim([-15,15])
    plt.ylim([-15,15])
    # draw_picture.draw_tubianxing(c,color='red')
    plt.plot([0,0],[-10,10])
    plt.plot([-10,10],[0,0])
    plt.show()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    start_time=time.time()*1000
    main()
    end_time=time.time()*1000
    print('time ',(end_time-start_time))

