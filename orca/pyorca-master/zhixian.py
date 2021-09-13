import math

#找点到线段最近的点
def proj_from_point_to_segment(point,line):
    abx=line[1][0]-line[0][0]
    aby=line[1][1]-line[0][1]
    apx=point[0]-line[0][0]
    apy=point[1]-line[0][1]
    ab_ap=abx*apx+aby*apy
    distab2=abx*abx+aby*aby
    dx=line[0][0]
    dy=line[0][1]
    if distab2!=0:
        t=ab_ap/distab2
        if t>=1:
            dx=line[1][0]
            dy=line[1][1]
        elif t>0:
            dx=line[0][0]+t*abx
            dy=line[0][1]+t*aby
        else:
            dx=line[0][0]
            dy=line[0][1]

    # proj_x=dx-point[0]
    # proj_y=dy-point[1]

    return dx,dy




