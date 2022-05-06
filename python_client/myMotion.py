# -*- coding: utf-8 -*-
"""
Created on Tue Oct  1 11:56:31 2019

@author: Mohammad Safeea

Test PTP motion class
"""

from iiwaPy import iiwaPy
import time
import math
# Connect to the robot
ip='192.168.1.2'
#ip='localhost'
iiwa=iiwaPy(ip)
iiwa.setBlueOn()
time.sleep(2)
iiwa.setBlueOff()   


# read some data from the robot
try:
    block = 20
    jp_home = [-1.0763223535892237, -0.44021275839332974, -0.19490759890874956, 1.9718060489813947, -2.5340529472480378, -0.9019241377011151, 2.4207385893638307]
    vel=[150]
    vRel=[0.5]
    iiwa.movePTPJointSpace(jp_home, vRel)
    up = 20
    origo=iiwa.getEEFPos()
    oX = origo[0]
    oY = origo[1]
    oZ = origo[2]
    print('Origo', origo)
    def drawLine(targets):
        for x1, y1, x2, y2 in targets:
            abovetarget = [oX + block * x1, oY + block * y1, oZ + up] + origo[3:]
            target = [oX + block * x1, oY + block * y1, oZ] + origo[3:]
            target2 = [oX + block * x2, oY + block * y2, oZ] + origo[3:]
            abovetarget2 = [oX + block * x2, oY + block * y2, oZ + up] +origo[3:]
            print('abovetaarget', abovetarget)
            iiwa.movePTPLineEEF(abovetarget,vel)
            iiwa.movePTPLineEEF(target,vel)
            iiwa.movePTPLineEEF(target2,vel)
            iiwa.movePTPLineEEF(abovetarget2,vel)
            

    def drawCirc(start, radius, deg):
        x1, y1 = start
        abovetarget = [oX + block * x1, oY + block * y1, oZ + up] + origo[3:]
        target = [oX + block * x1, oY + block * y1, oZ] + origo[3:]
        center = [oX + block * x1, oY +  y1 - radius * block]
        ang = deg * math.pi/180
        iiwa.movePTPLineEEF(abovetarget,vel)
        iiwa.movePTPLineEEF(target,vel)
        iiwa.movePTPArcXY_AC([ang], center, vel)

    print("My tests")
    
   
    #Change this!!!!
   
    # Get current cartezian position
    T = [(6, 6, 0, 6), (3, 6, 3, 0)]
    A = [(7, 6, 5, 0), (7, 6, 9, 0), (8, 2, 6, 2)]
    K = [(15, 6, 15, 0), (18, 6, 15, 2), (16, 3, 18, 0)]
    Cstart = (13, 6)
    Cradius = 3
    excl = [(20, 6, 20, 1)]
    dotStart = (20, 0)
    
    abovestart = [oX, oY, oZ + up] +origo[3:]
    iiwa.movePTPLineEEF(abovestart,vel)
    print('T')
    drawLine(T)
    
    print('A')

    drawLine(A)
    exit()
    print('C')
    drawCirc(Cstart, Cradius, 180)
    print('K')
    drawLine(K)
    print('!')
    drawLine(excl)
    print('.')
    drawCirc(dotStart, 0.5, 360)

    print("Done!")
    '''
    iiwa.movePTPLineEEF(cPos,vel)
    ang = 180 * math.pi/180
    iiwa.movePTPArcXY_AC([ang], center, vel)
    #iiwa.movePTPLineEEF(cPos,vel)
    '''


    
except Exception as e:
    print('an error happened')
    print(e)
    
iiwa.close()