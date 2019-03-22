#!/usr/bin/env python
import scipy.interpolate
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import math
xf = 0.005
xb = -0.01
y1 = -0.23
y2 = -0.30
y3 = -0.33
xstep = 0.1
ystep = (y1-y2)/2
ystep_down = (y2-y3)/2
#leg_l=-math.sqrt(math.pow(xstep/2,2)+math.pow(y2,2))
#y3=-math.sqrt(math.pow(leg_l,2)-math.pow(xstep/4,2))
leg_l = y2-0.02

pos1=[
    [[xf-xstep/4,y2+ystep*3/2],    [xb+xstep/2,y2],                [xf+xstep/2,y2],                    [xb-xstep/4,y2+ystep*3/2]],
    [[xf,y1],                      [xb+xstep/3,y2-ystep_down/3],   [xf+xstep/3,y2-ystep_down/3],       [xb,y1]],
    [[xf+xstep/4,y2+ystep*3/2],    [xb+xstep/6,y2-ystep_down*2/3], [xf+xstep/6,y2-ystep_down*2/3],     [xb+xstep/4,y2+ystep*3/2]],
    [[xf+xstep/2,y2],              [xb,y3],                        [xf,y3],                            [xb+xstep/2,y2]]
]
pos2=[[pos1[i][2],pos1[i][3],pos1[i][0],pos1[i][1]]for i in range(4)]
DEFAULT_POS=pos1
# DEFAULT_POS = [
# 				[[xf-2*xstep,y1] for i in range(4)],
# 				[[xf - 2 * xstep, y1] for i in range(4)],
# 				[[xf - 2 * xstep, y1] for i in range(4)]
                #  [[xb+xstep/2,y2+ystep]for i in range(4)],
               #   [[xb+xstep,y1]for i in range (4)],
               #   [[xb+3*xstep/2,y2+ystep]for i in range (4)],
               #   [[xb+2*xstep,y2]for i in range(4)]
               # ]
# DEFAULT_POS = [[[-0.03,-0.27] for i in range(4)],
# 				[[0.03,-0.27] for i in range(4)],
# 				[[0.06,-0.3] for i in range(4)]]
FREQUENCY = 50
RUN_TIME = 0.1
target_RF = [xf-xstep/2, y2]
target_RB = [xb+xstep/2, y2]
target_LF = [xf+xstep/2, y2]
target_LB = [xb-xstep/2, y2]
def MoveLegsP2P(target_pos=DEFAULT_POS,frequency = FREQUENCY):
    '''
    target_pos = [ [[x1,y1],[x2,y2],[x3,y3],[x4,y4]],
                    [[x1,y1],[x2,y2],[x3,y3],[x4,y4]],
                     ....	]
    T_STAND = 0.2           #float is necessary
    STEP_FREQUENCY = 25     #float is necessary
    '''
    target_pos = list(target_pos)
    if len(target_pos) == 1:
        mode = 'linear'
    else:
        # mode = 'cubic'
        mode = 'linear'

    # target_pos.insert(0,pos1[3])
    target_pos.insert(0, [target_RF, target_RB, target_LF, target_LB])
    # target_pos.insert(0,[[0,0]for i in range(4)])
    #target_pos = [ [[x1,y1],[x2,y2],[x3,y3],[x4,y4]],  actual pos
    #				[[x1,y1],[x2,y2],[x3,y3],[x4,y4]],
    #				[[x1,y1],[x2,y2],[x3,y3],[x4,y4]],
    #			     ....	]

    x = [[item[i][0] for item in target_pos] for i in range(4)]
    y = [[item[i][1] for item in target_pos] for i in range(4)]
    # x = [[x1,x1,x1,x1...],[x2,x2,x2,x2...],[x3,x3,x3,x3...],[x4,x4,x4,x4...]]
    # print x
    # print y
    functs = [0,0,0,0]
    for i in range(4):
    	if (x[i].count(x[i][1])>=len(x[i])-1 or y[i].count(y[i][1])>=len(y[i])-1):
    		functs[i] = interp1d(x[i], y[i], kind='linear')
    	else:
    		functs[i] = interp1d(x[i], y[i], kind=mode)
    # functs = [interp1d(x[i],y[i],kind = 'linear')if (x[i].count(x[i][1])>=len(x[i])-1 or y[i].count(y[i][1])>=len(y[i])-1)
    #           else interp1d( [x[i][0],x[i][-1]],[y[i][0],y[i][-1]],kind = mode) for i in range(4)]
    # functs = [f1,f2,f3,f4]

    xxs = [np.linspace(target_pos[0][i][0],
    target_pos[-1][i][0], frequency) for i in range(4)]
    #xxs = [[x1,x1,x1,x1...],[x2,x2,x2,x2...],[x3,x3,x3,x3...],[x4,x4,x4,x4...]]

    yys = [functs[i](xxs[i]) for i in range(4)]
    #yys = [[y1,y1,y1,y1...],[y2,y2,y2,x2...],[x3,x3,x3,x3...],[x4,x4,x4,x4...]]

    plt.plot(xxs[2], yys[2])
    plt.show()


if __name__=="__main__":
    t = 0
    T = 1.0
    spin_rate = 0.01
    frequency = int(T/4/spin_rate)
    step_length = 0.1
    step_height = 0.2
    min_height = -0.4
    xs = [-step_length/2,-step_length*3/8,0,step_length*3/8,step_length/2]
    ys = [min_height,min_height+step_height*2/4,min_height+step_height,min_height+step_height*2/4,min_height]
    funct = interp1d(xs, ys, kind='cubic')
    xss = np.linspace(-step_length/2,step_length/2,frequency)
    yss = funct(xss)
    plt.plot(xss,yss)
    plt.show()
# MoveLegsP2P()

