import matplotlib.pyplot as plt

import math
# import draw_picture
import time


def Minkowski_sum(rec1, rec2):
    vectors=[]
    ans=[]

    #
    tmp_rec=rec1
    init_point1=rec1[0]
    for i in range(len(tmp_rec)):
        point=tmp_rec[i]
        if init_point1[1]==point[1] and init_point1[0]>point[0]:
            init_point1=point
        if point[1]>init_point1[1]:
            init_point1=point
        if i>0:
            vectors.append([tmp_rec[i][0] - tmp_rec[i - 1][0], tmp_rec[i][1] - tmp_rec[i - 1][1]])
        if i==len(tmp_rec)-1:
            vectors.append([tmp_rec[0][0] - tmp_rec[i][0], tmp_rec[0][1] - tmp_rec[i][1]])

    tmp_rec=rec2
    init_point2=rec2[0]
    for i in range(len(tmp_rec)):
        point=tmp_rec[i]
        if init_point2[1]==point[1] and init_point2[0]>point[0]:
            init_point2=point
        if point[1]>init_point2[1]:
            init_point2=point
        if i>0:
            vectors.append([tmp_rec[i][0] - tmp_rec[i - 1][0], tmp_rec[i][1] - tmp_rec[i - 1][1]])
        if i==len(tmp_rec)-1:
            vectors.append([tmp_rec[0][0] - tmp_rec[i][0], tmp_rec[0][1] - tmp_rec[i][1]])

    tmp_point=[init_point1[0]+init_point2[0],init_point1[1]+init_point2[1]]
    ans.append([tmp_point[0],tmp_point[1]])


    tmp_cmp=[]
    for i in range(len(vectors)):
        tmp_cmp.append([math.atan2(vectors[i][1],vectors[i][0]),vectors[i]])
    tmp_cmp=sorted(tmp_cmp,key= lambda tmp:tmp[0])
    # for i in tmp_cmp:
    #     print(i[0],i[1].x,i[1].y)
    vectors=[]
    for i in range(1,len(tmp_cmp)):
        if tmp_cmp[i][0]==tmp_cmp[i-1][0]:
            tmp_cmp[i][1][0]+=tmp_cmp[i-1][1][0]
            tmp_cmp[i][1][1]+=tmp_cmp[i-1][1][1]
        else:
            vectors.append(tmp_cmp[i-1][1])
    vectors.append(tmp_cmp[len(tmp_cmp)-1][1])


    for i in range(len(vectors)-1):
        tmp_point[0]+=vectors[i][0]
        tmp_point[1]+=vectors[i][1]
        ans.append([tmp_point[0],tmp_point[1]])

    return ans

def __test1():
    a=[]
    a.append([0,0])
    a.append([2,3])
    a.append([-3,1])

    b=[]
    b.append([0,0])
    b.append([1,1])
    b.append([0,4])
    b.append([-2,3])

    c=Minkowski_sum(a,b)

    plt.xlim([-10,10])
    plt.ylim([-10,10])
    draw_picture.draw_tubianxing(a)
    draw_picture.draw_tubianxing(b)
    draw_picture.draw_tubianxing(c,color='red')
    plt.show()

def __test2():
    a=[]
    a.append([-2,1])
    a.append([-2,2])
    a.append([-2.5,3])
    a.append([-4,5])
    a.append([-6,3])
    a.append([-3,1])

    b=[]
    b.append([1,2])
    b.append([3,5])
    b.append([-3,5])
    b.append([-1,2])

    c=Minkowski_sum(a,b)
    print(c)
    # plt.xlim([-10,10])
    # plt.ylim([-10,10])
    # draw_picture.draw_tubianxing(a)
    # draw_picture.draw_tubianxing(b)
    # draw_picture.draw_tubianxing(c,color='red')
    # plt.plot([0,0],[-10,10])
    # plt.plot([-10,10],[0,0])
    # plt.show()

def main(name):
    __test2()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    start_time=time.time()*1000
    main('PyCharm')
    end_time=time.time()*1000
    print('time ',(end_time-start_time))
