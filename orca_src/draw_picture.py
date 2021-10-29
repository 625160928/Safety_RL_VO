import matplotlib.pyplot as plt

"""
输入 n*2的list
【
[x1,y1],
[x2,y2],
[x3,y3],
...
[xn,yn]
】

"""
def draw_tubianxing(points_list,color='black'):
    xx=[]
    yy=[]
    for i in range(len(points_list)):
        xx.append(points_list[i][0])
        yy.append(points_list[i][1])
    xx.append(points_list[0][0])
    yy.append(points_list[0][1])
    plt.plot(xx,yy,color)