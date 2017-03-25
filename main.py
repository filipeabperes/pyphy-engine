from cairo_trial import *
import random
from datetime import datetime
import math
import os
import sys
import argparse

def randBallPos(xSz=256, ySz=256, wThick=5, numBall=10, ballRadius=5,tableType=0):
    # random initialize multiple balls without overlapping
    numLeft=numBall
    pos=[(0,0)]*numBall
    tmp_st=[x+ballRadius for x in wThick]
    tmp_ran=(xSz-2*tmp_st[0],ySz-2*tmp_st[1])
    while numLeft!=0:
        done=False
        while not done:
            tmp_pos = (tmp_st[0]+random.random()*tmp_ran[0],tmp_st[1]+random.random()*tmp_ran[1])
            done=True
            for bid in range(numBall-numLeft):
                dist = (tmp_pos[0]-pos[bid][0])**2+(tmp_pos[1]-pos[bid][1])**2
                if dist<4*ballRadius**2:
                    done=False
                    break
        pos[numBall-numLeft]=(tmp_pos[0],tmp_pos[1])
        numLeft-=1
    return pos

def createExp(numBall=8,tableType=0,ballR=10):
    # create table
    xSz = ySz = 256
    world = pm.World(xSz=xSz, ySz=ySz)
    # add walls
    if tableType==0: # square
        wThick = [5,5]
    elif tableType==1: # rectangle
        wThick=[14,71]
    xLength, yLength = 256, 256-wThick[1]
    xLeft, yTop = 0, 0
    wallHorDef = pm.WallDef(sz=gm.Point(xLength, wThick[1]), fColor=pm.Color(0.5,0.5,0.5))
    wallVerDef = pm.WallDef(sz=gm.Point(wThick[0], yLength), fColor=pm.Color(0.5,0.5,0.5))
    world.add_object(wallVerDef, initPos=gm.Point(xLeft, yTop))
    world.add_object(wallVerDef, initPos=gm.Point(xLeft + xLength -wThick[0], yTop))
    world.add_object(wallHorDef, initPos=gm.Point(xLeft, yTop))
    world.add_object(wallHorDef, initPos=gm.Point(xLeft, yTop + yLength))

    # add balls
    ballRadius = ballR
    bPos = randBallPos(xSz=xSz, ySz=ySz, wThick=wThick, numBall=numBall+1, ballRadius=ballRadius,tableType=tableType)
    bCue  = pm.BallDef(fColor=pm.Color(0.0,0.0,1.0), radius=ballRadius, sThick=0)
    #bCue  = pm.BallDef(fColor=pm.Color(1.0,0.0,0.0), radius=ballRadius, sThick=0)
    world.add_object(bCue, initPos=gm.Point(bPos[-1][0],bPos[-1][1]))

    bOther = [None]*numBall
    for i in range(numBall):
        bOther[i]  = pm.BallDef(fColor=pm.Color(0.0,0.0,1.0), radius=ballRadius, sThick=0)
        world.add_object(bOther[i], initPos=gm.Point(bPos[i][0],bPos[i][1]))

    im = world.generate_image()
    return im, world

def saveExp(Iout='./image_%04d.jpg', Lout=['./test_p.txt','./test_v.txt'],numBall=8,aF=200,aD=0,tableType=0,ballR=10,ballM=1,Pinit=[]):
    im,world = createExp(numBall,tableType,ballR)
    model = pm.Dynamics(world,aFriction=aF,aColDamp=aD)
    # set initial speed
    #v0=600;vDelta=200;
    #v0=400;vDelta=200;
    v0=500;vDelta=500;
    numF=100
    # set mass
    for bb in model.world_.dynamic_.keys():
        model.world_.dynamic_[bb].set_mass(ballM)

    if len(Pinit)<=1: # random init
        for bb in model.world_.dynamic_.keys():
            vv = [(2*random.randint(0,1)-1)*v0+(2*random.randint(0,1)-1)*vDelta*random.random() for x in range(2)]
            model.world_.dynamic_[bb].set_velocity(gm.Point(vv[0],vv[1]))
        if len(Pinit)==1:
            for j in range(int(Pinit)):
                im = ball_world_step(j, model, False)

    else: # from initialzation
        #initialize from previous
        a=open(Pinit[0]);aa=a.readlines();pp=[float(x) for x in aa[-1].split(',')];a.close()
        a=open(Pinit[1]);aa=a.readlines();vv=[float(x) for x in aa[-1].split(',')];a.close()
        for k,bb in enumerate(model.world_.dynamic_.keys()):
            model.world_.dynamic_[bb].set_velocity(gm.Point(vv[k*2],vv[k*2+1]))
            model.world_.dynamic_[bb].set_position(gm.Point(pp[k*2],pp[k*2+1]))
    if Lout!=None:
        ll_p = open(Lout[0],'w')
        ll_v = open(Lout[1],'w')
	vv=[None]*len(model.world_.dynamic_)*2
	pp=[None]*len(model.world_.dynamic_)*2
    im=None
    doIm = Iout is not None
    for i in range(numF):
        im = ball_world_step(i, model, doIm)
        #print "step",i
        if doIm:
            if im is None:
                print "im is None"
                break;
            name = Iout  % (i+1)
            scm.imsave(name, im)
        if Lout!=None:
            for bid,bb in enumerate(model.world_.dynamic_.keys()):
                vel = model.get_object_velocity(bb)
                vv[bid*2] = '%.2f' % vel.x()
                vv[bid*2+1] = '%.2f' % vel.y()
                pos = model.get_object_position(bb)
                pp[bid*2] = '%.2f' % pos.x()
                pp[bid*2+1] = '%.2f' % pos.y()
            ll_p.write(",".join(pp)+'\n')
            ll_v.write(",".join(vv)+'\n')
    if Lout!=None:
        ll_p.close()
        ll_v.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='billiard ball physics simulation')
    parser.add_argument('--seed', default=1, type=int)
    parser.add_argument('--aDamp', default=0, type=float)
    parser.add_argument('--aFric', default=0, type=float)
    parser.add_argument('--ballNum', default=1, type=int) # number of balls besides the cue ball
    parser.add_argument('--ballRadius', default=10, type=int)
    parser.add_argument('--ballMass', default=1, type=float)
    # []: random init; '5': burn-in steps; '[file_pos,file_vel]': read the initial pos and vel
    parser.add_argument('--ballInit', default="", type=str)
    parser.add_argument('--tableType', default=0, type=int)
    parser.add_argument('--outDir', default='./result/', type=str)
    parser.add_argument('--outIm', default=False, type=bool)    

    param = parser.parse_args()

    random.seed(param.seed)
    outN = param.outDir + '/exp%d-%d_%d_%d' %(param.tableType, param.ballMass, param.ballNum, param.ballRadius)+'_'+str(param.aDamp)+'_'+str(param.aFric)+'/'
    if not os.path.isdir(outN):
        os.makedirs(outN) # in case of folder heirarchy

    fns = ['pos/','vel/','im/']
    for fn in fns:
        if not os.path.isdir(outN+fn):
            os.mkdir(outN+fn)    
    fn2 = '%06d' % param.seed
    logN = [outN+'pos/'+fn2+'_p.txt', outN+'vel/'+fn2+'_v.txt'];
    imN=None
    if param.outIm:        
        if not os.path.isdir(outN+'im/'+fn2):
            os.mkdir(outN+'im/'+fn2)
        imN = outN+'im/'+fn2+'/image_%04d.png'
    
    
    saveExp(imN,logN,param.ballNum,param.aFric,param.aDamp,param.tableType,param.ballRadius,param.ballMass,param.ballInit)
