# -*- coding: utf-8 -*-

import vrep
import numpy as np

l1 = 0.5
l2 = 0.35
l3 = 0.15

X = lambda t1,t2,t3: [
     [l1*np.sin(t1) + l2*np.sin(t1 + t2) + l3*np.sin(t1 + t2 + t3)],
     [0.0],
     [l1*np.cos(t1) + l2*np.cos(t1 + t2) + l3*np.cos(t1 + t2 + t3)]
     
     ]

Vec = lambda t1,t2,t3: [
            [t1],
            [t2],
            [t3]
        ]

J = lambda t1,t2,t3: [
            [
                l1*np.cos(t1) + l2*np.cos(t1 + t2) + l3*np.cos(t1 + t2 + t3),
                l2*np.cos(t1 + t2) + l3*np.cos(t1 + t2 + t3),
                l3*np.cos(t1 + t2 + t3)
            ],
            [
                0,0,0        
            ],
            [
                -l1*np.sin(t1) - l2*np.sin(t1 + t2) - l3*np.sin(t1 + t2 + t3),
                -l2*np.sin(t1 + t2) - l3*np.sin(t1 + t2 + t3),
                -l3*np.sin(t1 + t2 + t3),
            ]
        ]

J_inv = lambda t1,t2,t3: np.linalg.pinv(J(t1,t2,t3))

points = [i for i in np.arange(-np.pi/3,np.pi/3, 0.05)]
radius = 0.8

cartesian_points = [(radius*1.25*np.sin(i), 0, radius*1.25*np.cos(i)) for i in points]

def set_velocity(handle, velocity, maximum=20):
        if np.abs(velocity) > maximum:
            if velocity > 0:
                velocity = maximum
            else:
                velocity = -maximum
                
        vrep.simxSetJointTargetVelocity(
                clientID,
                handle,
                velocity,
                vrep.simx_opmode_streaming
                )
        if velocity == None:
            velocity = 0
        
        return velocity

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',20000,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')
    
    _, j1 = vrep.simxGetObjectHandle(clientID,'J1',vrep.simx_opmode_blocking)
    _, j2 = vrep.simxGetObjectHandle(clientID,'J2',vrep.simx_opmode_blocking)
    _, j3 = vrep.simxGetObjectHandle(clientID,'J3',vrep.simx_opmode_blocking)
    
    idx = 0
    for p in cartesian_points: 
        _, t1 = vrep.simxGetJointPosition(clientID, j1,vrep.simx_opmode_blocking)
        _, t2 = vrep.simxGetJointPosition(clientID, j2,vrep.simx_opmode_blocking)
        _, t3 = vrep.simxGetJointPosition(clientID, j3,vrep.simx_opmode_blocking)
        
        deg = lambda x: x*180/np.pi
        
        X_cur = X(t1,t2,t3)
        X_des = Vec(*p)
        err = np.subtract(X_des,X_cur)*0.6
    
        T = (J_inv(t1,t2,t3)@err)
        
        v = []
        
        v.append(set_velocity(j1,T[0][0], 0.4))
        v.append(set_velocity(j2,T[1][0], 0.4))
        v.append(set_velocity(j3,T[2][0], 0.4))
        
        idx += 1
        print(idx)
          
    set_velocity(j1,0,0)   
    set_velocity(j2,0,0)   
    set_velocity(j3,0,0)   
    
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

#%%



