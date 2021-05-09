#--------------------- Import Librariesc -------------------------------------------------|
from controller import Robot,Keyboard
import numpy as np
import random
from gym import Env
from gym.spaces import Discrete,Box
#import tensorflow as tf
#---------------------- Initilise Robot --------------------------------------------------|
robot = Robot()
timestep = int(robot.getBasicTimeStep())

#---------------------- Initilize Devices ------------------------------------------------|

keyboard=Keyboard()
keyboard.enable(timestep)

m0 = robot.getDevice('m0')
m1 = robot.getDevice('m1')
m2 = robot.getDevice('m2')
m3 = robot.getDevice('m3')
m4 = robot.getDevice('m4')

p0 = robot.getDevice('p0')
p1 = robot.getDevice('p1')
p2 = robot.getDevice('p2')
p3 = robot.getDevice('p3')
p4 = robot.getDevice('p4')

c1 = robot.getDevice('c1')

gps = robot.getDevice('gps')

#--------------------- Set initial Values -----------------------------------------------|

p0.enable(timestep)
p1.enable(timestep)
p2.enable(timestep)
p3.enable(timestep)
p4.enable(timestep)
c1.enablePresence(timestep)
gps.enable(timestep)

m0.setVelocity(1)
m1.setVelocity(1)
m2.setVelocity(1)
m3.setVelocity(1)
m4.setVelocity(1)



#-------------------- Pre Functions -----------------------------------------------------|

def IsMotionComplete(mi0,mi1,mi2,mi3,mi4,fact): #check if motion has compleated
    if mi0*(1-fact) <= p0.getValue() <= mi0*(fact+1):
        if mi1*(1-fact) <= p1.getValue() <= mi1*(fact+1):
            if mi2*(1-fact) <= p2.getValue() <= mi2*(fact+1):
                if mi3*(1-fact) <= p3.getValue() <= mi3*(fact+1):
                    if mi4*(1-fact) <= p4.getValue() <= mi4*(fact+1):
                        return True
                    else:
                        return False
                else:
                    return False
            else:
                return False        
        else:
            return False
    else:
        return False
    


def PosInit():#used for setting initial random values to motor
    mv0 = np.random.uniform(0,6.2831853072)
    mv1 = np.random.uniform(0,2.0943951024)
    mv2 = np.random.uniform(0,2.0943951024)
    mv3 = np.random.uniform(0,2.0943951024)
    mv4 = np.random.uniform(0,2.0943951024)
    c =  random.randint(0, 1)
    
    # check if connection is possible
    islockable = c1.getPresence()
        
    m0.setPosition(mv0)
    m1.setPosition(mv1)
    m2.setPosition(mv2)
    m3.setPosition(mv3)
    m4.setPosition(mv4)
    
    if c == 1:
        c1.lock()
    else:
        c1.unlock()    
    
    
    while not IsMotionComplete(mv0, mv1, mv2, mv3, mv4, 0.001):
        if robot.step(timestep) == -1:
            break
    
    pos = gps.getValues()
    print(c1.isLocked())         
    return [mv0,mv1,mv2,mv3,mv4,pos[0],pos[1],pos[2],c,islockable]



def updateRobot(instate):
    m0.setPosition(instate[0])
    m1.setPosition(instate[1])
    m2.setPosition(instate[2])
    m3.setPosition(instate[3])
    m4.setPosition(instate[4])
    if instate[0] == 0 and c1.isLocked():
        c1.unlock()
    elif instate[0] == 1 and not c1.isLocked():
        c1.lock()

    
    




#----------------- Environment class ----------------------------------------------------|
class tars(Env):
    def __init__(self):
        # Action we can take increse angle ,decrease angle,stop x 5 servos and last conncet or not
        self.action_space = Box(low=np.array([-1.0, -1.0,-1.0,-1.0,-1.0,0]), high=np.array([1.0, 1.0, 1.0, 1.0, 1.0]), dtype=np.intc)
       
        #Observationspace                          b m1,m2,m3,m4 px,   py  pz c  ilk  pick pos, drop pos 
        self.observation_space = Box(low=np.array([0, 0,0, 0, 0, -0.7,-0.7,-0.7,0,0, -1,-1,-1,-1,-1,-1]),high=np.array([6.2831853072, 2.0943951024, 2.0943951024, 2.0943951024,2.0943951024,0.7,0.7,0.7,1,1,1,1,1,1,1,1]), dtype=np.float32)                     
        # Set start positions
        self.state = PosInit() #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!CHECK IF POS REMOVAL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # Set episod length
        self.episod_length = 1875 #ie 1875 x 32ms = 60 sec
        
        
    def step(self,action):        
        self.episod_length -=1
        #apply action to state variables action
        self.state[0] += action[0]
        self.state[1] += action[1]
        self.state[2] += action[2]
        self.state[3] += action[3]
        self.state[4] += action[4]
        
        if action[5] >0 :
            self.state[8] = 1
        elif action[5]<1 :
            self.state[8] = 0
            
        #Apply state to robot
        updateRobot(self.state)
        
        #Update robot
        robot.step(timestep)
        
        # get observation
        
        #calculate reward
        
        
        
        
                
        
            
            
        
        
    def render(self):
        pass
    def reset(self):
        pass
     


print(PosInit())

while robot.step(timestep) != -1:
    
    pass
    
    
        
        
        
        
        