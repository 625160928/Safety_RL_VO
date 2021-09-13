import matplotlib.pyplot as plt
def draw_tubianxing(points_list,color='black'):
    xx=[]
    yy=[]
    for i in range(len(points_list)):
        xx.append(points_list[i][0])
        yy.append(points_list[i][1])
    xx.append(points_list[0][0])
    yy.append(points_list[0][1])
    plt.plot(xx,yy,color)