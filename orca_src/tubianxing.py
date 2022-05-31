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

def Is_rec_collide(rec1, rec2):
    for i in range(1,len(rec1)):
        for j in range(1,len(rec2)):

            if Is_line_collide(rec1[i-1:i+1],rec2[j-1:j+1]):
                # print(rec1[i-1:i+1],rec2[j-1:j+1])
                return True

    if Is_line_collide([rec1[0],rec1[-1]],[rec2[0],rec2[-1]]):
        # print(rec1[-1:1],rec2[-1:+1])
        return True

    return False

def get_func_of_line(line):
    if line[0][0]==line[1][0]:
        return 1,0,-line[0][0]
    if line[0][1]==line[1][1]:
        return 0,1,-line[0][1]
    k=(line[1][1]-line[0][1])/(line[1][0]-line[0][0])
    b=line[1][1]-k*line[1][0]
    return k,-1,b

def Get_line_collide(a1,b1,c1,a2,b2,c2):
    if a1==0:
        y=-c1/b1
        x=-(c2+b2*y)/a2
        return [x,y]
    if a2==0:
        y=-c2/b2
        x=-(c1+b1*y)/a1
        return [x,y]
    if b1==0:
        x=-c1/a1
        y=-(c2+a2*x)/b2
        return [x,y]
    if b2==0:
        x=-c2/a2
        y=-(c1+a1*x)/b1
        return [x,y]
    y=(a2*c1-a1*c2)/(a1*b2-a2*b1)
    x=-(c1+b1*y)/a1

    return [x, y]

def Is_line_collide(line1,line2):

    a1,b1,c1=get_func_of_line(line1)
    a2,b2,c2=get_func_of_line(line2)

    if a1==a2 and b2==b1 and c1==c2:
        if a1==0:
            x_min1=min(line1[0][0],line1[1][0])
            x_max1=max(line1[0][0],line1[1][0])
            x_min2=min(line2[0][0],line2[1][0])
            x_max2=max(line2[0][0],line2[1][0])
            if x_max1<x_min2 or x_min1>x_max2:
                return False
        else:
            y_min1=min(line1[0][1],line1[1][1])
            y_max1=max(line1[0][1],line1[1][1])
            y_min2=min(line2[0][1],line2[1][1])
            y_max2=max(line2[0][1],line2[1][1])
            if y_max1<y_min2 or y_min1>y_max2:
                return False

        return True

    if a1==a2 and c1!=c2:
        return False

    point=Get_line_collide(a1,b1,c1,a2,b2,c2)

    if line1[0][0]!=line1[1][0]:
        if point[0]<min(line1[0][0],line1[1][0]) or point[0]>max(line1[0][0],line1[1][0]):
            return False
    else:
        if point[1]<min(line1[0][1],line1[1][1]) or point[1]>max(line1[0][1],line1[1][1]):
            return False

    if line2[0][0] != line2[1][0]:
        if point[0] < min(line2[0][0], line2[1][0]) or point[0] > max(line2[0][0], line2[1][0]):
            return False
    else:
        if point[1] < min(line2[0][1], line2[1][1]) or point[1] > max(line2[0][1], line2[1][1]):
            return False


    return True

def atest_rec_collide():
    c = [[-7, 10], [-9, 8], [-7, 5], [-4, 3], [-1, 3], [1, 6], [1, 7], [0.5, 8], [-1.0, 10]]
    c2=[[0,0],[3,0],[3,3],[-0,4]]
    # print(Is_rec_collide(c,c2))
    tmp_x=[]
    tmp_y=[]
    for x,y in c:
        tmp_x.append(x)
        tmp_y.append(y)
    tmp_x.append(c[0][0])
    tmp_y.append(c[0][1])
    plt.plot(tmp_x,tmp_y)


    tmp_x=[]
    tmp_y=[]
    for x,y in c2:
        tmp_x.append(x)
        tmp_y.append(y)
    tmp_x.append(c2[0][0])
    tmp_y.append(c2[0][1])
    plt.plot(tmp_x,tmp_y)


    plt.xlim([-15, 15])
    plt.ylim([-15, 15])
    # draw_picture.draw_tubianxing(c,color='red')
    plt.show()


def old_test():
    c = [[-7, 10], [-9, 8], [-7, 5], [-4, 3], [-1, 3], [1, 6], [1, 7], [0.5, 8], [-1.0, 10]]
    check_point = [3, 10]

    get_min_point_in_tubianxing(check_point, c)

    print(IsPointInConvexPolygon(c, check_point))

    plt.scatter(check_point[0], check_point[1])
    plt.xlim([-15, 15])
    plt.ylim([-15, 15])
    # draw_picture.draw_tubianxing(c,color='red')
    plt.plot([0, 0], [-10, 10])
    plt.plot([-10, 10], [0, 0])
    plt.show()

def main():
    atest_rec_collide()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    start_time=time.time()*1000
    main()
    end_time=time.time()*1000
    print('time ',(end_time-start_time))

