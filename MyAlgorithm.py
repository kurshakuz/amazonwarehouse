import numpy as np
import cv2
import math

import sys

import threading
import time
from datetime import datetime
from ompl_solution.Point2DPlanning import Plane2DEnvironment
import sys
from purePursuit import State, calc_target_index, pure_pursuit_control

time_cycle = 80


def clearscreen(numlines=10):
    """Clear the console.
    numlines is an optional argument used only as a fall-back.
    """
    import os
    if os.name == "posix":
        # Unix/Linux/MacOS/BSD/etc
        os.system('clear')
    elif os.name in ("nt", "dos", "ce"):
        # DOS/Windows
        os.system('CLS')
    else:
        # Fallback for other operating systems.
        print '\n' * numlines



class MyAlgorithm(threading.Thread):

    def __init__(self, grid, sensor, vel):
        self.sensor = sensor
        self.grid = grid
        self.vel = vel
        sensor.getPathSig.connect(self.generatePath)

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

        self.curr_state = State()
        self.prev_state = State()

        self.targetV = 0.5
        # Pure_pursuit related variables
        self.cx = []
        self.cy = []
        self.target_ind = 0
        self.lastIndex = 0
        self.yawCorrected = False
        self.posCorrected = False
        self.currV = 0
        self.prevV = 0
        self.prevPositionalError = 0
        self.currPositionalError = 0
        self.dt = 0
        self.robotXInRobotFrame = 0
        self.robotYInRobotFrame = 0
        self.robotThetaInRobotFrame = 0

        self.gotoPointChecked = False

        # Debugging Info Variables
        self.printK_V = 0
        self.printK_W = 0
        self.printCurrV = 0
        self.printCurrW = 0
        self.printYawDiff = 0


    def run (self):

        while (not self.kill_event.is_set()):
           
            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)
            self.dt = ms/1000

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()

    """ Write in this method the code necessary for looking for the shorter
        path to the desired destiny.
        The destiny is chosen in the GUI, making double click in the map.
        This method will be call when you press the Generate Path button. 
        Call to grid.setPath(path) method for setting the path. """
    def generatePath(self, list):
        print("LOOKING FOR SHORTER PATH")
        mapIm = self.grid.getMap()      
        dest = self.grid.getDestiny()   
        gridPos = self.grid.getPose()

        size = mapIm.shape
        h = size[0]
        w = size[1]
        print (h)
        print (w)
        mapppm = np.zeros((h,w,3))
        kernel = np.ones((15,15),np.uint8)
        erosion = cv2.erode(mapIm,kernel,iterations = 1) 
        #cv2.imshow('image',erosion)
        for i in range(h):
            for j in range(w):
                mapppm[i,j,0] = erosion[i,j]
                mapppm[i,j,1] = erosion[i,j]
                mapppm[i,j,2] = erosion[i,j]

        
        cv2.imwrite('map_image.ppm',mapppm)

        planner = str(list[0])
        #objective = str(list[1])
        runtime = str(list[1])

        print planner
        #print objective
        print runtime

        fname = sys.path[0]+'/map_image.ppm'
        print fname
        env = Plane2DEnvironment(fname)

        if env.plan(gridPos[0],gridPos[1], dest[0],dest[1],planner,float(runtime)):
            env.recordSolution()
            env.save("result_demo.ppm")
            pathlist = [[] for i in range(2)]
            pathlist = env.getPath()
            worldPathList = [[] for i in range(2)]


            patharray = np.array(pathlist)
            patharray = np.rint(patharray)
            #print patharray
            size = patharray.shape
            #print size
            num = 0
            pathX_old = -1
            pathY_old = -1
            for i in range(size[1]):
                pathX = patharray[0][i]
                pathY = patharray[1][i]
                if pathX != pathX_old or pathY != pathY_old:
                    self.grid.setPathVal(int(pathX), int(pathY), num)
                    tmp = self.grid.gridToWorld(pathX, pathY)
                    worldPathList[0].append(tmp[0])
                    worldPathList[1].append(tmp[1])
                    num += 1
                    pathX_old = pathX
                    pathY_old = pathY

            self.grid.setPathFinded()
            worldPathArraySmoothTmp = np.array(worldPathList)
            #sigma = 3
            #orldPathArraySmoothTmp[0] = gaussian_filter1d(worldPathArraySmoothTmp[0], sigma)
            #worldPathArraySmoothTmp[1] = gaussian_filter1d(worldPathArraySmoothTmp[1], sigma)
            worldPathArraySmoothTmp[0] = worldPathArraySmoothTmp[0]
            worldPathArraySmoothTmp[1] = worldPathArraySmoothTmp[1]

            print worldPathArraySmoothTmp
            self.grid.setWorldPathArray(worldPathArraySmoothTmp)
            self.initializePurePursuit()

        #Represent the Gradient Field in a window using cv2.imshow


    def initializePurePursuit(self):
        worldPathArrayExecuting = self.grid.getWorldPathArray()
        self.cx = worldPathArrayExecuting[0]
        self.cy = worldPathArrayExecuting[1]
        # Obtain the current position of the vehicle
        x = self.sensor.getRobotX()
        y = self.sensor.getRobotY()
        yaw = self.sensor.getRobotTheta()
        self.curr_state = State(x, y, yaw, v=0.0, w=0.0)
        self.prev_state = State(x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0)

        # Find the goal point
        # Calculate the point on the path closest to the vehicle
        # We already have a certain look-ahead distance_diff (Lfc)
        # search look ahead target point index
        self.target_ind = calc_target_index(self.curr_state, self.cx, self.cy)
        self.lastIndex = len(self.cx) - 1
        self.computeNextState()

        print("Pure Persuit Initialized")


    def computeNextState(self):
        clearscreen()
        print "prev_x: " + str(self.prev_state.x)
        print "prev_y: " + str(self.prev_state.y)
        print "prev_yaw: " + str(self.prev_state.yaw)
        print "req_x: " + str(self.curr_state.x)
        print "req_y: " + str(self.curr_state.y)
        print "req_yaw: " + str(self.curr_state.yaw)
        print "After Computation:"

        '''self.ai = PIDControl(self.targetV, self.prev_state.v)
        self.di, self.target_ind = pure_pursuit_control(self.prev_state, self.cx, self.cy, self.target_ind)
        self.prev_state = State(self.curr_state.x, self.curr_state.y, self.curr_state.yaw, self.curr_state.r,
                                self.curr_state.phi, self.curr_state.v, self.curr_state.w)
        self.curr_state = update_state(self.curr_state, self.ai, self.di)'''
        prevTargetIdx = self.target_ind
        print "prev_target_ind: " + str(prevTargetIdx)

        self.di, self.target_ind = pure_pursuit_control(self.prev_state, self.cx, self.cy, self.target_ind)
        tempYaw = math.atan2(self.curr_state.y - self.prev_state.y,  self.curr_state.x - self.prev_state.x)
        self.prev_state = State(self.curr_state.x, self.curr_state.y, self.curr_state.yaw,
                                self.curr_state.v, self.curr_state.w)
        self.curr_state = State(self.cx[self.target_ind], self.cy[self.target_ind], tempYaw)


        #print "ai: " + str(self.ai)
        #print "di: " + str(self.di)
        print "prev_x: " + str(self.prev_state.x)
        print "prev_y: " + str(self.prev_state.y)
        print "prev_yaw: " + str(self.prev_state.yaw)
        print "req_x: " + str(self.curr_state.x)
        print "req_y: " + str(self.curr_state.y)
        print "req_yaw: " + str(self.curr_state.yaw)
        print "target_ind: " + str(self.target_ind)
        time.sleep(0.5)

    def TransformationMatrix2D(self):
        RT = np.matrix(
            [[math.cos(self.sensor.getRobotTheta()), -math.sin(self.sensor.getRobotTheta()), self.sensor.getRobotX()],
             [math.sin(self.sensor.getRobotTheta()), math.cos(self.sensor.getRobotTheta()), self.sensor.getRobotY()],
             [0, 0, 1]])
        return RT

    def worldToRobot2D(self, x, y):
        orig_poses = np.matrix([[x], [y], [1]])
        final_poses = np.linalg.inv(self.TransformationMatrix2D()) * orig_poses
        xInRobotFrame = final_poses.flat[0]
        yInRobotFrame = final_poses.flat[1]
        return (xInRobotFrame, yInRobotFrame)

    def computeThetaInRobotFrame(self, robotFramex, robotFrameY):
        theta = math.atan2(robotFrameY,robotFramex)
        return theta


    def printDebugInfo(self):
        clearscreen()
        if self.yawCorrected:
            print "Yaw Corrected!"
        else:
            print "Yaw Not Corrected!"
        if self.posCorrected:
            print "Position Corrected!"
        else:
            print "Position Not Corrected!"

        print "K_V: " + str(self.printK_V)
        print "K_W: " + str(self.printK_W)
        print "curr_vel: " + str(self.printCurrV)
        print "last_ind: " + str(self.lastIndex)
        print "target_ind: " + str(self.target_ind)
        print "curr_speed: " + str(self.curr_state.v)
        print "curr_x: "  + str(self.sensor.getRobotX())
        print "curr_y: "  + str(self.sensor.getRobotY())
        print "req_x: "  + str(self.curr_state.x)
        print "req_y: "  + str(self.curr_state.y)
        print "prev_x: "  + str(self.prev_state.x)
        print "prev_y: "  + str(self.prev_state.y)

        print "correcting robot yaw:"
        print "current yaw: " + str(self.sensor.getRobotTheta())
        print "yaw_diff: " + str(self.printYawDiff)
        print "curr_W: " + str(self.printCurrW)


    def correctYawInRobotFrame(self):
        self.vel.setV(0)
        self.vel.setW(0)
        self.printCurrV = 0
        self.printCurrW = 0
        self.yawCorrected = False
        curr_w = 0
        x1, y1 = self.worldToRobot2D(self.curr_state.x, self.curr_state.y)
        e = self.computeThetaInRobotFrame(x1, y1)
        req_e = 0.05
        if abs(e) > req_e:
            self.vel.setV(0)
            alpha = 0.4
            #K = 0.5*(1 - math.exp(-alpha*math.pow(e,2)/abs(e)))
            K = 0.5
            self.printYawDiff = e
            self.printK_W = K
            if e > req_e and e <= math.pi:
                if e - math.pi/2 < math.pi/2:
                    curr_w = K * e
                else:
                    curr_w = -K * e
            if -math.pi <= e < -req_e:
                if e - math.pi/2 < math.pi/2:
                    curr_w = K * e
                else:
                    curr_w = -K * e
            self.vel.setW(curr_w)
            self.printCurrW = curr_w
        if abs(e) <= req_e:
            #print "Robot yaw corrected."
            self.vel.setW(0)
            self.printCurrW = 0
            self.yawCorrected = True
            K = 0
            self.printK_W = K

    def correctPosInRobotFrame(self):
        self.vel.setV(0)
        self.vel.setW(0)
        self.printCurrV = 0
        self.printCurrW = 0
        self.posCorrected = False
        x1, y1 = self.worldToRobot2D(self.curr_state.x, self.curr_state.y)
        dist = math.sqrt(x1**2 + y1**2)
        req_dist= 0.01
        if abs(dist) > req_dist:
            self.vel.setV(0)
            alpha = 0.9
            #K = 5*(1 - math.exp(-alpha*math.pow(dist,2)/abs(dist)))
            K = 0.3
            self.printDistanceDiff = dist
            self.printK_V = K
            self.vel.setV(K * dist)
            self.printCurrV = K * dist
        if abs(dist) <= req_dist:
            #print "Robot yaw corrected."
            self.vel.setV(0)
            self.printCurrV = 0
            self.posCorrected = True
            K = 0
            self.printK_V = K

    def moveOnPath(self):
        self.correctYawInRobotFrame()
        if self.yawCorrected:
            self.correctPosInRobotFrame()
        if self.yawCorrected and self.posCorrected and self.lastIndex != self.target_ind:
            self.computeNextState()
        if self.yawCorrected and self.posCorrected and self.lastIndex == self.target_ind:
            print "last Index Reached!"
            self.vel.setV(0)
            self.vel.setW(0)
        else:
            self.printDebugInfo()
            self.yawCorrected = False

    def setGotoPointFlag(self, isChecked):
        self.gotoPointChecked = isChecked

    def gotoPoint(self):
        dest = self.grid.getDestiny()
        x = dest[0]
        y = dest[1]
        rX, rY = self.grid.gridToWorld(x, y)
        self.curr_state.x = rX
        self.curr_state.y = rY
        self.target_ind = self.lastIndex
        self.correctYawInRobotFrame()
        if self.yawCorrected:
            self.correctPosInRobotFrame()
        if self.yawCorrected and self.posCorrected:
            print "GoToPoint Reached!"
            self.vel.setV(0)
            self.vel.setW(0)
            self.gotoPointChecked = False
        else:
            self.printDebugInfo()
            self.yawCorrected = False

    """ Write in this method the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """
    def execute(self):
        if self.gotoPointChecked:
            self.gotoPoint()
        else:
            self.moveOnPath()
