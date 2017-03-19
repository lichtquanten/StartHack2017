from distutils.log import Log

import paho.mqtt.client as mqtt
import json
import math as m
import numpy as np
import time

SERVER = "127.0.0.1"
#SERVER = "10.10.10.30"
PORT = 1883

PLAYER_NAME = "TheRegressor"
DISTANCE_BETWEEN_WHEELS = 25210.14298575622/90
CONVERT_COUNT_DIST = 3/10

GAME_STATE = 0 # 0 is waiting, 1 is playing
firstState = True
firstGameData = True
game_data = {}
game_log = []
i = 0
differences = []

WAIT_FOR_EXEC_FLAG = False


class Robot(object):
    """docstring for ClassName"""
    def __init__(self):
        print("init")
        self.x = 640
        self.y = 480
        self.rightCount = 0
        self.leftCount = 0
        self.angle = 0
        self.targetAngle = 0
        self.startTurnTime = time.time()
        self.lastMotorMovement = time.time()
        self.lastLeftMotorValue = 0
        self.lastRightMotorValue = 0

        self.firstCheckArrival = True

        self.goodPoints = np.zeros(shape=(45, 2))
        self.badPoints = np.zeros(shape=(5, 2))
        self.targetIndices = list(range(45))  # can be reordered
        self.targetPoint = [0,0]
        self.lastPoints = []

        self.beginAt = time.time()

    def bestNextPoint(self):
        totalArray = np.zeros(shape=(np.size(self.goodPoints,0), 1))
        for i in range(np.size(self.goodPoints,0)):
            temp = self.goodPoints + np.negative(self.goodPoints[i])
            #print("TEMP1: " + str(temp))
            temp = np.square(temp)
            #print("TEMP2: " + str(temp))
            #print("TEMP SHAPE" + str(temp.shape))
            k = np.array([[1],[1]])
            #print("K SHAPE" + str(k.shape))
            #print(temp)
            temp = np.dot(temp, np.array([[1],[1]]))
            #print("TEMP3: " + str(temp))
            distanceToStartPoint = m.sqrt(np.dot(self.goodPoints[i] - [self.x, self.y], self.goodPoints[i] - [self.x, self.y]))
            #print("DISTANCE: " + str(distanceToStartPoint))
            totalArray[i] = sum(temp) + 4*distanceToStartPoint
        self.targetPoint = list(self.goodPoints[np.argmin(totalArray)])

        def handleMoving(self, distance):
            a = (self.angle + 5 * 360) % 360
            print("A: " + str(a))
            b = (self.targetAngle + 5 * 360) % 360
            print("B: " + str(b))
            if abs(b - a) > 270 or abs(b - a) < 90:
                if b > a:
                    self.turnRight(int(b - a) % 360)
                else:
                    self.turnLeft(int(abs(b - a)) % 360)
                # while abs(self.angle - b) > 40 and abs(self.angle - b) < 320:
                #     print(abs(self.angle - b))
                self.moveForward(int(distance))
            else:
                if b > a:
                    if (b - 180) > a:
                        self.turnRight(int((b - 180) - a) % 360)
                    else:
                        self.turnLeft(int(a - (b - 180)) % 360)
                else:
                    if (b + 180) < a:
                        self.turnLeft(int(a - (b + 180)) % 360)
                    else:
                        self.turnRight(int((b + 180) - a) % 360)

                # while abs(self.angle - b) > 40 and abs(self.angle - b) < 320:
                #     print(abs(self.angle - b))
                self.moveBackward(int(distance))

    # Mouvements
    def moveForward(self, value):
        client.publish('robot/process', '{"command": "forward", "args": ' + str(value) + '}', qos=2, retain=False)

    def moveBackward(self, value):
        client.publish('robot/process', '{"command": "backward", "args": ' + str(value) + '}', qos=2, retain=False)

    def turnRight(self, degree):
        client.publish('robot/process', '{"command": "right", "args": ' + str(degree) + '}', qos=2, retain=False)

    def turnLeft(self, degree):

        client.publish('robot/process', '{"command": "left", "args": ' + str(degree) + '}', qos=2, retain=False)

    def stop(self):
        client.publish('robot/process/', '{"command": "stop"}', qos=2, retain=False)

    def setOdometry(self, vRight, vLeft):
        dRight = vRight - self.rightCount
        dLeft = vLeft - self.leftCount
        dCenter = (dRight+dLeft)*CONVERT_COUNT_DIST/2
        dAngle = (dLeft-dRight)/DISTANCE_BETWEEN_WHEELS
        self.angle = (self.angle + dAngle * 180 / m.pi) % 360
        self.x = self.x + dCenter*m.cos(self.angle*m.pi/180)
        self.y = self.y - dCenter*m.sin(self.angle*m.pi/180)

        self.rightCount = vRight
        self.leftCount = vLeft

    # Trig functions
    def setTargetAngle(self, xTarget, yTarget):
        print("setTargetAngle")
        xDiff = xTarget - self.x
        yDiff = yTarget - self.y
        self.targetAngle = abs(m.degrees(m.atan(yDiff/xDiff)))
        if yDiff < 0:
            if xDiff < 0:
                self.targetAngle = 180 - self.targetAngle
        else:
            if xDiff > 0:
                self.targetAngle = 360 - self.targetAngle
            else:
                self.targetAngle = 180 + self.targetAngle
        self.targetAngle += 5
        print("targ ang:" + str(self.targetAngle))

    # def getNearestNeighbour(self):
    #     # self.currentTarget
    #     startingPoint = [self.x, self.y]
    #     minDistance = 100000
    #     indexToKill = None
    #     for index in self.targetIndices:
    #         distance = (np.sqrt(np.dot((self.goodPoints[index]-startingPoint),(self.goodPoints[index]-startingPoint))))*100
    #         if distance < minDistance:
    #             indexToKill = index
    #             minDistance = distance
    #     print("INDEX TO KILL" + str(indexToKill))
    #     self.targetPoint = self.goodPoints[indexToKill]
    #

    def checkArrival(self, points):
        if self.firstCheckArrival:
            print("FIRST CHECK ARRIVAL")
            self.lastPoints = list(points)
            print(str(self.lastPoints))
            self.firstCheckArrival = False
        goodCounter2 = -1
        for i in range(len(points)):
            if points[i]["score"] == 1:
                goodCounter2 += 1
                if points[i]["collected"] != self.lastPoints[i]["collected"]:
                    print("WE HERE FAMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM")
                    point = [points[i]["x"], points[i]["y"]]
                    originalDims = list(self.goodPoints.shape)
                    temp = np.zeros(shape=(originalDims[0], 2))
                    # index = np.where(self.goodPoints == [points[i]["x"], points[i]["y"]])
                    # print("INDEX: " + str(index))
                    subCounter = 0
                    for k in range(originalDims[0]):
                        print(self.goodPoints[k])
                        print(list(self.goodPoints[k]))
                        if list(self.goodPoints[k]) != point:
                            temp[subCounter] = self.goodPoints[k]
                            subCounter += 1
                    self.goodPoints = temp

                    # self.goodPoints = np.delete(self.goodPoints, [points[i]["x"], points[i]["y"]])
                    # print("NEW: " + str(self.goodPoints.size))
                    # print("NEW: " + str(self.goodPoints))
                    # print("CHK ARRIVAL")
                    self.bestNextPoint()
                    self.setNextPath()
        self.lastPoints = list(points)

    def setNextPath(self):
        print("setNextPATHH")
        self.stop()
        xT = self.targetPoint[0]
        print("xT:" + str(xT))
        yT = self.targetPoint[1]
        print("yT:" + str(yT))
        self.setTargetAngle(xT,yT)
        distanceToTarget = m.sqrt((xT - self.x) * (xT - self.x) + (yT - self.y) * (yT - self.y))
        self.handleMoving(int(distanceToTarget/CONVERT_COUNT_DIST))

    def handleMoving(self, distance):
        a = (self.angle + 5 * 360) % 360
        print("A: " + str(a))
        b = (self.targetAngle + 5 * 360) % 360
        print("B: " + str(b))
        if abs(b - a) > 270 or abs(b - a) < 90:
            if b > a:
                self.turnRight(int(b - a) % 360)
            else:
                self.turnLeft(int(abs(b - a)) % 360)
            # while abs(self.angle - b) > 40 and abs(self.angle - b) < 320:
            #     print(abs(self.angle - b))
            print("FORWARD" + str(distance))
            self.moveForward(int(distance))
        else:
            if b > a:
                if (b - 180) > a:
                    self.turnRight(int((b - 180) - a) % 360)
                else:
                    self.turnLeft(int(a - (b - 180)) % 360)
            else:
                if (b + 180) < a:
                    self.turnLeft(int(a - (b + 180)) % 360)
                else:
                    self.turnRight(int((b + 180) - a) % 360)

            # while abs(self.angle - b) > 40 and abs(self.angle - b) < 320:
            #     print(abs(self.angle - b))
            print("BACKWARD" + str(distance))
            self.moveBackward(int(distance))

    def checkHeading(self):
        if time.time() - self.lastMotorMovement > 1.5:
            print("MOTOR MOVE 2 LONG")
            #self.getNearestNeighbour()
            print("CHK HEADING")
            self.bestNextPoint()
            self.setNextPath()
            self.lastMotorMovement = time.time()
        # if time.time() - self.startTurnTime > 30:
        #     print("ANGLE 2 LONG")
        #     if abs(self.angle - self.targetAngle) > 30:
        #         print("angle is whackkk")
        #         self.setNextPath()
        #     self.startTurnTime = time.time()

    def getRemainingTime(self):
        timeSpentInMs = time.time() - self.beginAt
        remainingTime = 120000 - timeSpentInMs

        if remainingTime < 0:
            remainingTime = 0

        return remainingTime

    def generateTargets(self, points):
        goodCounter = 0
        badCounter = 0
        for point in points:
            if point["score"] == 1:
                self.goodPoints[goodCounter] = [point["x"], point["y"]]
                goodCounter += 1
            else:
                self.badPoints[badCounter] = [point["x"], point["y"]]
                badCounter += 1
        return self.goodPoints

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe('players/' + PLAYER_NAME + '/#')
    client.subscribe('robot/state')
    client.subscribe('robot/error')
    client.subscribe('players/' + PLAYER_NAME + '/incoming')
    client.subscribe('players/' + PLAYER_NAME + '/game')
    client.publish('players/' + PLAYER_NAME , '{"command": "register"}')


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global GAME_STATE
    global firstState
    global firstGameData
    global game_log
    global i
    global WAIT_FOR_EXEC_FLAG
    #print(msg.topic)
    obj = json.loads(msg.payload.decode("utf-8"))
    #print(obj)

    game_log.append(obj)


    if GAME_STATE == 2:
        if msg.topic == 'robot/state':
            robot.setOdometry(obj['right_motor'], obj['left_motor'])
            if firstState:
                #robot.getNearestNeighbour()
                print("STATE")
                robot.bestNextPoint()
                robot.setNextPath()
                firstState = False
                robot.lastLeftMotorValue = obj["left_motor"]
                robot.lastRightMotorValue = obj["right_motor"]
            if m.floor(robot.lastLeftMotorValue) != m.floor(obj["left_motor"]) or m.floor(robot.lastRightMotorValue) != m.floor(obj["right_motor"]):
                robot.lastMotorMovement = time.time()
            robot.lastLeftMotorValue = obj["left_motor"]
            robot.lastRightMotorValue = obj["right_motor"]
            robot.checkHeading() #sees if we need to tell the robot to find a path again

        elif (msg.topic == 'players/%s/game' % PLAYER_NAME):
            print("********** STORED GAME_DATA ************")
            if firstGameData:
                robot.x = obj["robot"]["x"]
                robot.y = obj["robot"]["y"]
                firstGameData = False
            robot.checkArrival(obj["points"])

    elif GAME_STATE == 1:
        if (msg.topic == 'players/%s/game' % PLAYER_NAME):
            robot.generateTargets(obj["points"])
            GAME_STATE = 2



    elif (GAME_STATE == 0):
        if ((msg.topic=='players/%s/incoming' % PLAYER_NAME) and ("command" in obj)):
            if (obj['command'] == "start"):
                print("********** RECEIVED START FROM SERVER ************")
                client.publish('players/' + PLAYER_NAME , '{"command": "start"}')
                GAME_STATE = 1
            elif (obj['command'] == "finished"):
                print("********** RECEIVED FINISHED FROM SERVER ************")
                GAME_STATE = 0
                client.disconnect()
                exit()


    i += 1
    if (i>=10):
        i = 0
        with open("data.txt","w") as f: #in write mode
            f.write("{}".format(game_log))



if __name__ == '__main__':

    robot = Robot()
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(SERVER, PORT, 60)

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    try:
        client.loop_forever()
        print("Looping")
    except (KeyboardInterrupt, SystemExit):
        print("Tearing down...")

        client.disconnect()
        raise
