import pygame
import numpy as np
import math
import time
import matplotlib.pyplot as plt

class Button:
    def __init__(self, coordinates, text):
        self.coordinates = coordinates
        self.text = text

    def setText(self, text):
        self.text = text

    def render(self):
        button = font.render(f'{self.text}', True, (255, 255, 255))
        WIN.blit(button, (self.coordinates[0], self.coordinates[1]))

    def click(self):
        if self.coordinates[0]<x<self.coordinates[0]+300 and self.coordinates[1]<y<self.coordinates[1]+30 and event.type==pygame.MOUSEBUTTONDOWN:
            return True
        
def newCompute(P0, P1, P2, P3):
    for i in range(0, 1000):
        t = i/1000
        x = valueCalculate(t, P0[0], P1[0], P2[0], P3[0])
        y = valueCalculate(t, P0[1], P1[1], P2[1], P3[1])
        pointsList.append((x,y))

def reCompute(P0, P1, P2, P3, startVal):
    for i in range(0, 1000):
        t = i/1000
        x = valueCalculate(t, P0[0], P1[0], P2[0], P3[0])
        y = valueCalculate(t, P0[1], P1[1], P2[1], P3[1])
        pointsList[i+startVal] = (x,y)

def adjacentCompute(iVal, points):
    if iVal-1>=0 and iVal+1<len(points):
        reCompute(points[iVal][1], points[iVal][2], points[iVal+1][0], points[iVal+1][1], iVal*1000)
        reCompute(points[iVal-1][1], points[iVal-1][2], points[iVal][0], points[iVal][1], (iVal-1)*1000)
    elif iVal-1>=0:
        reCompute(points[iVal-1][1], points[iVal-1][2], points[iVal][0], points[iVal][1], (iVal-1)*1000)
    else:
        reCompute(points[iVal][1], points[iVal][2], points[iVal+1][0], points[iVal+1][1], iVal*1000)

def valueCalculate(t, P0, P1, P2, P3):
    a = np.array([
        [1, t, t**2, t**3]
    ])
    b= np.array([
        [1, 0, 0, 0],
        [-3, 3, 0, 0],
        [3, -6, 3, 0],
        [-1, 3, -3, 1]
    ])
    c= np.array([
        [P0],
        [P1],
        [P2],
        [P3]
    ])
    product1 = np.matmul(a,b)
    result = np.matmul(product1, c)[0][0]
    return result

def distanceCalc():
    totaldistance = 0
    segmentDistance = []
    for i in range(len(pointsList)-1):
        segLength = math.sqrt((pointsList[i][0]-pointsList[i+1][0])**2+(pointsList[i][1]-pointsList[i+1][1])**2)
        totaldistance+=segLength
        if (i+2)%1000==0:
            segmentDistance.append((totaldistance/60))
    totaldistance /= 60

    for i in range(1, len(segmentDistance)):
        segmentDistance[-i] = segmentDistance[-i]-segmentDistance[-i-1]
    return totaldistance, segmentDistance

def linVelGen(distance, maxDistance, maxVel, accel):
    slope = (76.576-3.828)/accel
    maxVel = 76.576*(maxVel/100)
    accelDistance = (maxVel-3.828)/slope

    max = True
    if accelDistance*2>maxDistance:
        accelDistance = maxDistance/2
        maxVel = slope*accelDistance+3.828

    if distance<accelDistance:
        velocity = distance*slope+3.828
    elif accelDistance<distance<maxDistance-accelDistance and max:
        velocity = maxVel
    else:
        velocity = maxVel+(maxDistance-accelDistance-distance)*slope

    return velocity

def curvatureCalc(t, P0, P1, P2, P3):
    vel = np.array([
        [0, 1, 2*t, 3*(t**2)]
    ])
    accel = np.array([
        [0, 0, 2, 6*t]
    ])
    b= np.array([
        [1, 0, 0, 0],
        [-3, 3, 0, 0],
        [3, -6, 3, 0],
        [-1, 3, -3, 1]
    ])
    xVals= np.array([
        [P0[0]/60],
        [P1[0]/60],
        [P2[0]/60],
        [P3[0]/60]
    ])
    yVals = np.array([
        [P0[1]/60],
        [P1[1]/60],
        [P2[1]/60],
        [P3[1]/60]
    ])
    xVel = np.matmul(vel, b)
    xVel = np.matmul(xVel, xVals)[0][0]

    xAccel = np.matmul(accel, b)
    xAccel = np.matmul(xAccel, xVals)[0][0]

    yVel = np.matmul(vel, b)
    yVel = np.matmul(yVel, yVals)[0][0]

    yAccel = np.matmul(accel, b)
    yAccel = np.matmul(yAccel, yVals)[0][0]
    # xProduct = np.matmul(b, xVals)
    # yProduct = np.matmul(b, yVals)

    # xVel = np.matmul(xProduct, vel)
    # yVel = np.matmul(yProduct, vel)

    # xAccel = np.matmul(xProduct, accel)
    # yAccel = np.matmul(yProduct, accel)

    derMatrix = np.array([
        [xVel, xAccel],
        [yVel, yAccel]
    ])

    curvature = np.linalg.det(derMatrix)/((math.sqrt(xVel**2+yVel**2))**3)*12
    return curvature

def trajectoryCalc(maxVel, accel):
    timeStamps = []
    angularVel = []
    linearVel = []
    t = 0
    tim = 0
    s = 0
    dT = 0.010

    totalDistance, segmentDistance = distanceCalc()

    v = 0
    w = 0
    while t<len(points)-1:

        v = linVelGen(s, totalDistance, maxVel, accel)
        curvature = curvatureCalc(t%1, points[int(t//1)][1], points[int(t//1)][2], points[int(t//1)+1][0], points[int(t//1)+1][1])
        w = curvature*v

        if curvature == 0:
            w = 0
        else:
            radius = 1/curvature
            w = v*curvature

            if curvature<0:
                if 10.4*w+76.576>v:
                    w = 76.576/(radius-10.4)
                    v = w*radius
            else:
                if -10.4*w+76.576>v:
                    w = 76.576/(radius+10.4)
                    v = w*radius
        
        s+=(v*dT)/12
        tim+=dT

        t=s/segmentDistance[int(t//1)]
        
        timeStamps.append(tim)
        angularVel.append(w)
        linearVel.append(v)
    graphVel(timeStamps, linearVel)








def graphVel(x, y):
    fig, ax = plt.subplots()
    ax.plot(x, y)
    plt.show()




WIDTH, HEIGHT = 1020, 720
WIN = pygame.display.set_mode((WIDTH, HEIGHT))

maxAngularVel = 421.87
accel = 1.5
maxVel = 100

pygame.font.init()
font = pygame.font.Font(None, 36)

run = True

bg = pygame.image.load("Vex Field.png")



pointsList = []

activeCircle = None
circleType = ""

points = [[[150, 300], [50, 300], [150, 300], False], [[200, 50], [300, 50], [200, 50], False]]
newCompute(points[0][1], points[0][0], points[1][0], points[1][1])

coordinateEdit = 0
xChange = False
setX = ""

yChange = False
setY = ""

velChange = False
setVel = ""

xVal = Button((750, 80), f'x: {round(points[coordinateEdit][1][0]/60,2)}')
yVal = Button((750, 110), f'y: {round((720-points[coordinateEdit][1][1])/60,2)}')
maximumVelocity = Button((750, 200), f"Max Velocity: {maxVel}%")
compute = Button((800, 400),"Compute")
while run:
    WIN.fill("black")
    WIN.blit(bg, (0, 0))
    pygame.draw.lines(WIN, (255, 255, 255), False, pointsList, width=3)

    xVal.setText(f'x: {round(points[coordinateEdit][1][0]/60,2)}')
    xVal.render()

    yVal.setText(f'y: {round((720-points[coordinateEdit][1][1])/60,2)}')
    yVal.render()

    pointNum = font.render(f'Point {coordinateEdit+1}', True, (255, 255, 255))
    WIN.blit(pointNum, (750, 40))

    startHeading = font.render(f"Start Heading: {round(math.atan2((points[0][0][1]-points[0][1][1]), (points[0][0][0]-points[0][1][0]))*180/math.pi+90)}", True, (255, 255, 255))
    WIN.blit(startHeading, (750, 150))

    maximumVelocity.setText(f"Max Velocity: {maxVel}%")
    maximumVelocity.render()

    compute.render()

    for i in range(len(points)):
        for a in range(3):
            if a == 0 or a == 2:
                pygame.draw.circle(WIN, "white", points[i][a], 5)
            else:
                pygame.draw.circle(WIN, (3, 198, 252), points[i][1], 8)

        pygame.draw.line(WIN, "white", points[i][0], points[i][1], width=2)
        pygame.draw.line(WIN, "white", points[i][2], points[i][1], width=2)

    x = pygame.mouse.get_pos()[0]
    y = pygame.mouse.get_pos()[1]

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                for i in range(len(points)):
                    for a in range(3):
                        if points[i][a] != None and math.sqrt((points[i][a][0]-x)**2+(points[i][a][1]-y)**2)<10:
                            activeCircle = [i, a]
                            aVal = a
                            iVal = i
                            coordinateEdit = i

        if xVal.click():
            xChange = True

        if xChange and event.type == pygame.KEYDOWN:
            setX+=event.unicode

        if yVal.click():
            yChange = True

        if yChange and event.type == pygame.KEYDOWN:
            setY+=event.unicode
        
        if maximumVelocity.click():
            velChange = True

        if velChange and event.type == pygame.KEYDOWN:
            setVel+=event.unicode

        if event.type == pygame.MOUSEMOTION:
            if activeCircle!=None:
                if aVal == 1:
                    points[iVal][0][0]+=x-points[iVal][1][0]
                    points[iVal][0][1]+=y-points[iVal][1][1]

                    points[iVal][2][0]+=x-points[iVal][1][0]
                    points[iVal][2][1]+=y-points[iVal][1][1]

                    points[iVal][1][0] = x
                    points[iVal][1][1] = y
                    
                    adjacentCompute(iVal, points)
                else:
                    if points[iVal][3] == False:
                        points[iVal][0][0] = x
                        points[iVal][0][1] = y

                        points[iVal][2][0] = x
                        points[iVal][2][1] = y
                    else:
                        if aVal == 0:
                            points[iVal][0][0] = x
                            points[iVal][0][1] = y

                            points[iVal][2][0] = points[iVal][1][0]+(points[iVal][1][0]-x)
                            points[iVal][2][1] = points[iVal][1][1]+(points[iVal][1][1]-y)
                        else:
                            points[iVal][2][0] = x
                            points[iVal][2][1] = y

                            points[iVal][0][0] = points[iVal][1][0]+(points[iVal][1][0]-x)
                            points[iVal][0][1] = points[iVal][1][1]+(points[iVal][1][1]-y)

                    adjacentCompute(iVal, points)

        if compute.click():
            trajectoryCalc(maxVel, accel)
        if event.type == pygame.MOUSEBUTTONUP:
            activeCircle=None
    
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_n:
                points.append([[x, y-100], [x,y], [x, y-100], False])
                points[len(points)-2][3] = True

                points[len(points)-2][2][0] = points[len(points)-2][1][0]+(points[len(points)-2][1][0]-points[len(points)-2][0][0])
                points[len(points)-2][2][1] = points[len(points)-2][1][1]+(points[len(points)-2][1][1]-points[len(points)-2][0][1])

                newCompute(points[len(points)-2][1], points[len(points)-2][2], points[len(points)-1][0], points[len(points)-1][1])

        if event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
            if xChange:
                points[coordinateEdit][1][0] = float(setX)*60
                setX = ""
                adjacentCompute(coordinateEdit, points)
                xChange = False

            if yChange:
                points[coordinateEdit][1][1] = 720-float(setY)*60
                setY = ""
                adjacentCompute(coordinateEdit, points)
                yChange = False

            if velChange:
                maxVel = int(setVel)
                setVel = ""
                velChange = False


    pygame.display.flip()