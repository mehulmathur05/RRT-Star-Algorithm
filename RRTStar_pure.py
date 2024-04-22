import math
import cv2
import numpy as np 
import random

image = cv2.imread("Image1.png")
height, width, _ = np.shape(image)
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.neighbors = []
        self.children = []
        self.cost = 0

    def setNeighbors(self, nodeList, neighborRadius):
        for node in nodeList:
            print(node.getCoords())
            if (distance(node.getCoords(), self.getCoords()) < neighborRadius and node not in self.neighbors):
                self.neighbors.append(node)
                node.neighbors.append(self)

    def setParent(self, parent):
        self.parent = parent

    def setChild(self, child):
        self.children.append(child)

    def getCoords(self):
        return (self.x, self.y)
    
    def setCost(self, cost):
        self.cost = cost

def drawPaths(start):
    for node in start.children:
        cv2.line(image, start.getCoords(), node.getCoords(), (255, 0, 0), 2)
        drawPaths(node)

def retracePath(goal, start):
    if goal == start: return
    cv2.line(image, goal.getCoords(), goal.parent.getCoords(), (0, 255, 0), 3)
    retracePath(goal.parent, start)

for i in range(height):
    for j in range(width):
        if image[i][j][0] == 76 and image[i][j][1] == 177 and image[i][j][2] == 34:
            startNode = Node(j, i)
        elif image[i][j][0] == 36 and image[i][j][1] == 28 and image[i][j][2] == 237:
            goalNode = Node(j, i)

def distance(coord1, coord2):
    dist = math.sqrt(pow((coord1[0] - coord2[0]), 2) + pow((coord1[1] - coord2[1]), 2))
    return dist

image_ = np.zeros((height, width), np.uint8)
def feasible(image):
    feasiblePoints = []
    for i in range(height):
        for j in range(width):
            if any(image[i][j] != 0):
                feasiblePoints.append((j, i))
                image_[i][j] = 255

    return feasiblePoints


def randomPoint(feasiblePoints):
    randPt = random.randint(0, len(feasiblePoints) - 1)
    return (feasiblePoints[randPt])

def checkObstacle(coord1, coord2):
    if coord1[0] != coord2[0]:
        slope = (coord2[1]-coord1[1])/(coord2[0]-coord1[0])
        intercept = (coord1[1]*coord2[0] - coord2[1]*coord1[0])/(coord2[0]-coord1[0])
        
        if abs(coord1[0]-coord2[0]) > abs(coord1[1]-coord2[1]):   
            if coord1[0]>coord2[0]:
                temp = coord1
                coord1 = coord2
                coord2 = temp
                
            for i in range(coord1[0], coord2[0]-1):
                j = math.floor(slope*i) + math.floor(intercept)
                print(coord1, coord2, (i, j))
                if (i, j) == coord2:
                    return False
                elif image_[j,i] == 0:
                    return True
       
        else:
            if coord1[1]>coord2[1]:
                temp = coord1
                coord1 = coord2
                coord2 = temp
                
            for j in range(coord1[1], coord2[1]-1):
                i = (j-intercept)/slope
                print(coord1, coord2, (i, j))
                if (i, j) == coord2:
                    return False
                if image_[j,math.floor(i)] == 0:
                    return True

    if coord1[0] == coord2[0]:
        if coord1[1]>coord2[1]:
            temp = coord1
            coord1 = coord2
            coord2 = temp
        i = coord1[0]
        for j in range(coord1[1], coord2[1]-1):
            if image_[j,i] == 0:
                return True
            # elif (i,j) == coord2:
            #     return False  
    return False

def nearestNode(randPt, nodeList):
    minDist = 1e5
    found = False
    for node in nodeList:
        print(randPt, node.getCoords())
        if not checkObstacle(randPt, node.getCoords()):
            dist = distance(randPt, node.getCoords())
            if dist < minDist:
                minDist = dist
                nearest = node
                found = True
    if found: return nearest, minDist
    else : return 0, 0


def RRTStar(image, startNode, goalNode):
    nodeList = [startNode]
    feasiblePoints = feasible(image)
    while True:
        randPt = randomPoint(feasiblePoints)

        stepDist = 30

        nearest, dNearest = nearestNode(randPt, nodeList)
        if (dNearest == 0): 
            # print("Jj")
            continue
        # else: cv2.circle(image, randPt, 2, (0,255,0), -1)


        if (dNearest < stepDist):
            newNode = Node(int(randPt[0]), int(randPt[1]))
        else:
            theta = math.atan2((nearest.y - randPt[1]) , (nearest.x - randPt[0]))
            newNode = Node(int(nearest.x - stepDist*math.cos(theta)), int(nearest.y - stepDist*math.sin(theta)))
            # print("newNode", newNode.getCoords())

        nodeList.append(newNode)
        newNode.setNeighbors(nodeList, 2.5*stepDist)
        father = nearest
        newNode.setCost(father.cost + distance(father.getCoords(), newNode.getCoords()))
        for neighbor in newNode.neighbors:
            if (not checkObstacle(newNode.getCoords(), neighbor.getCoords())):
                cost = neighbor.cost + distance(newNode.getCoords(), neighbor.getCoords())
                if cost < newNode.cost:
                    father = neighbor
                    newNode.cost = cost
                
        newNode.setParent(father)
        father.setChild(newNode)

        if distance(newNode.getCoords(), goalNode.getCoords()) < stepDist:
            nodeList.append(goalNode)
            newNode.setNeighbors(nodeList, 2.5*stepDist)
            goalNode.setCost(newNode.cost + distance(newNode.getCoords(), goalNode.getCoords()))
            goalNode.setParent(newNode)
            newNode.setChild(goalNode)
            retracePath(goalNode, startNode)
            break 

        drawPaths(startNode)
        cv2.imshow("image", image)
        cv2.waitKey(1)
        cv2.destroyAllWindows()


RRTStar(image, startNode, goalNode)

cv2.imshow("image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
