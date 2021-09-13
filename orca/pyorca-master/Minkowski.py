import matplotlib.pyplot as plt

import math
import draw_picture
import time

import tubianxing


def Minkowski_sum(rec1, rec2):
    vectors=[]
    ans=[]

    if not tubianxing.IsDisOrder(rec1):
        rec1.reverse()
    if not tubianxing.IsDisOrder(rec2):
        rec2.reverse()

    #
    tmp_rec = rec1
    init_point1 = rec1[0]
    for i in range(len(tmp_rec)):
        point = tmp_rec[i]
        if init_point1[1] == point[1] and init_point1[0] > point[0]:
            init_point1 = point
        if point[1] > init_point1[1]:
            init_point1 = point
        if i > 0:
            vectors.append([tmp_rec[i][0] - tmp_rec[i - 1][0], tmp_rec[i][1] - tmp_rec[i - 1][1]])
        if i == len(tmp_rec) - 1:
            vectors.append([tmp_rec[0][0] - tmp_rec[i][0], tmp_rec[0][1] - tmp_rec[i][1]])

    tmp_rec = rec2
    init_point2 = rec2[0]
    for i in range(len(tmp_rec)):
        point = tmp_rec[i]
        if init_point2[1] == point[1] and init_point2[0] > point[0]:
            init_point2 = point
        if point[1] > init_point2[1]:
            init_point2 = point
        if i > 0:
            vectors.append([tmp_rec[i][0] - tmp_rec[i - 1][0], tmp_rec[i][1] - tmp_rec[i - 1][1]])
        if i == len(tmp_rec) - 1:
            vectors.append([tmp_rec[0][0] - tmp_rec[i][0], tmp_rec[0][1] - tmp_rec[i][1]])


    tmp_point=[init_point1[0]+init_point2[0],init_point1[1]+init_point2[1]]
    ans.append([tmp_point[0],tmp_point[1]])
    # print('start point ',tmp_point)

    tmp_cmp=[]
    for i in range(len(vectors)):
        atan=math.atan2(vectors[i][1],vectors[i][0])
        # print([atan,vectors[i]])
        # if atan<-math.pi:
        #     atan+=math.pi*2
        # if atan>math.pi:
        #     atan-=math.pi*2

        tmp_cmp.append([atan,vectors[i]])
    # tmp_cmp=sorted(tmp_cmp,key= lambda tmp:tmp[0],reverse=True)
    # for i in tmp_cmp:
    #     print('before sort ',i[0],i[1][0],i[1][1])
    # print('===========')
    tmp_cmp=sorted(tmp_cmp,key= lambda tmp:tmp[0])

    # for i in tmp_cmp:
        # print('after sort ',i[0],i[1][0],i[1][1])

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
    a.append([10,0])
    a.append([11,0])
    a.append([11,1])
    a.append([10,1])

    b=[]
    b.append([0,3])
    b.append([-10,0])
    b.append([0,-1])

    c=Minkowski_sum(a,b)

    plt.xlim([-10,20])
    plt.ylim([-10,20])
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

    # a.reverse()


    b=[]
    b.append([1,2])
    b.append([3,5])
    b.append([-3,5])
    b.append([-1,2])
    # b.reverse()

    print(a)
    print(b)
    c=Minkowski_sum(a,b)
    print(c)
    plt.xlim([-10,10])
    plt.ylim([-10,10])
    draw_picture.draw_tubianxing(a)
    draw_picture.draw_tubianxing(b)
    draw_picture.draw_tubianxing(c,color='red')
    plt.plot([0,0],[-10,10])
    plt.plot([-10,10],[0,0])
    plt.show()

def __test3():
    # a=[[ 8.43712576, 12.16196421],[30.80197503, 44.40048997],[  30.80197503, -178.55072587],[  2.51633575, -14.5865184 ]]
    # b=[[5.40591312, 2.06533619],[93.41457955, 35.6891626 ],[ 93.41457955, -35.6891626 ],[ 5.40591312, -2.06533619]]
    a=[[ 8.43712576, 12.16196421],[30.80197503, 44.40048997],[  30.80197503, -178.55072587]]

    b=[[5.40591312, 2.06533619],[93.41457955, 35.6891626 ],[ 93.41457955, -35.6891626 ]]
    a.reverse()
    b.reverse()

    c=Minkowski_sum(a,b)
    print(c)
    # plt.xlim([-10,10])
    # plt.ylim([-10,10])
    draw_picture.draw_tubianxing(a)
    draw_picture.draw_tubianxing(b)
    draw_picture.draw_tubianxing(c,color='red')
    plt.plot([0,0],[-10,10])
    plt.plot([-10,10],[0,0])
    plt.show()

def main(name):
    __test2()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    start_time=time.time()*1000
    main('PyCharm')
    end_time=time.time()*1000
    print('time ',(end_time-start_time))
