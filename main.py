import pygame
import numpy as np
import math
import time
import matplotlib.pyplot as plt

# Button Class
class Button:
    # Initialize
    def __init__(self, coordinates, text):
        self.coordinates = coordinates
        self.text = text

    # Set the button's text
    def setText(self, text):
        self.text = text

    # Render the button
    def render(self):
        button = font.render(f'{self.text}', True, (255, 255, 255))
        WIN.blit(button, (self.coordinates[0], self.coordinates[1]))

    # Determine if clicked
    def click(self):
        if self.coordinates[0]<x<self.coordinates[0]+300 and self.coordinates[1]<y<self.coordinates[1]+30 and event.type==pygame.MOUSEBUTTONDOWN:
            return True

# Calculate point coordinates
def valueCalculate(t, P0, P1, P2, P3):
    a = np.array([
        [1, t, t**2, t**3]
    ])
    # Bezier Matrix
    b= np.array([
        [1, 0, 0, 0],
        [-3, 3, 0, 0],
        [3, -6, 3, 0],
        [-1, 3, -3, 1]
    ])
    # Point Matrix
    c= np.array([
        [P0],
        [P1],
        [P2],
        [P3]
    ])
    # Multiply matrices and return results
    product1 = np.matmul(a,b)
    result = np.matmul(product1, c)[0][0]
    return result

# Compute a new spline
def newCompute(P0, P1, P2, P3):
    # Calculate 1000 points
    for i in range(0, 1000):
        t = i/1000
        x = valueCalculate(t, P0[0], P1[0], P2[0], P3[0])
        y = valueCalculate(t, P0[1], P1[1], P2[1], P3[1])
        splinePoints.append((x,y))

# Compute a spline change
def reCompute(P0, P1, P2, P3, startVal):
    for i in range(0, 1000):
        t = i/1000
        x = valueCalculate(t, P0[0], P1[0], P2[0], P3[0])
        y = valueCalculate(t, P0[1], P1[1], P2[1], P3[1])
        splinePoints[i+startVal] = (x,y)

# Determine how to recompute spline
def adjacentCompute(iVal, points):
    # If the point moving has 2 adjacent points, recompute the segments between all 3 points
    if iVal-1>=0 and iVal+1<len(points):
        reCompute(points[iVal][1], points[iVal][2], points[iVal+1][0], points[iVal+1][1], iVal*1000)
        reCompute(points[iVal-1][1], points[iVal-1][2], points[iVal][0], points[iVal][1], (iVal-1)*1000)

    # If the point is the last point, recompute the last segment
    elif iVal-1>=0:
        reCompute(points[iVal-1][1], points[iVal-1][2], points[iVal][0], points[iVal][1], (iVal-1)*1000)

    # Recompute the first segment
    else:
        reCompute(points[iVal][1], points[iVal][2], points[iVal+1][0], points[iVal+1][1], iVal*1000)

# Calculate the distance of the spline
def distanceCalc():
    totaldistance = 0
    segmentDistance = []

    # Go through the spline points amnd determine the distance travelled
    for i in range(len(splinePoints)-1):
        segLength = math.sqrt((splinePoints[i][0]-splinePoints[i+1][0])**2+(splinePoints[i][1]-splinePoints[i+1][1])**2)
        totaldistance+=segLength

        # If the segment is completed, add its value to segmentDistance
        if (i+2)%1000==0:
            segmentDistance.append((totaldistance/60))

    # Convert to feet
    totaldistance /= 60

    # Go through the segments and make calculate indiivdiual segment length
    for i in range(1, len(segmentDistance)):
        segmentDistance[-i] = segmentDistance[-i]-segmentDistance[-i-1]
    return totaldistance, segmentDistance

# Generate linear velocity
def linVelGen(distance, maxDistance, maxVel):

    # Determine the slope and maximum velocity
    slope = (maxLinVel-1)/accel
    maxVel = maxLinVel*(maxVel/100)

    # Determine how far the robot needs to go to reach the maximum velocity
    accelDistance = (maxVel-1)/slope

    max = True

    # If the segment is too small, follow a triangular profile
    if accelDistance*2>maxDistance:
        accelDistance = maxDistance/2
        maxVel = slope*accelDistance+1

    # First part of trapezoid (accelerating)
    if distance<accelDistance:
        velocity = distance*slope+1
    
    # Middle part of trapepzoid (steady velocity)
    elif accelDistance<distance<maxDistance-accelDistance and max:
        velocity = maxVel
    
    # End of trapezoid (decelerating)
    else:
        velocity = maxVel+(maxDistance-accelDistance-distance)*slope

    return velocity

# Calculate the curvature at a point
def curvatureCalc(t, P0, P1, P2, P3):
    # Velocity matrix (first derivative)
    vel = np.array([
        [0, 1, 2*t, 3*(t**2)]
    ])

    # Acceleration matrix (second derivative)
    accel = np.array([
        [0, 0, 2, 6*t]
    ])
    # Bezier matrix
    b= np.array([
        [1, 0, 0, 0],
        [-3, 3, 0, 0],
        [3, -6, 3, 0],
        [-1, 3, -3, 1]
    ])
    # x value matrix
    xVals= np.array([
        [P0[0]/60],
        [P1[0]/60],
        [P2[0]/60],
        [P3[0]/60]
    ])
    # y value matrix
    yVals = np.array([
        [P0[1]/60],
        [P1[1]/60],
        [P2[1]/60],
        [P3[1]/60]
    ])
    # Calculate x velocity
    xVel = np.matmul(vel, b)
    xVel = np.matmul(xVel, xVals)[0][0]

    # Calculate x acceleration
    xAccel = np.matmul(accel, b)
    xAccel = np.matmul(xAccel, xVals)[0][0]

    # Calculate y velocity
    yVel = np.matmul(vel, b)
    yVel = np.matmul(yVel, yVals)[0][0]

    # Calculate y acceleration
    yAccel = np.matmul(accel, b)
    yAccel = np.matmul(yAccel, yVals)[0][0]

    # Calculate determinant of matrix and get curvature
    derMatrix = np.array([
        [xVel, xAccel],
        [yVel, yAccel]
    ])
    curvature = np.linalg.det(derMatrix)/((math.sqrt(xVel**2+yVel**2))**3)

    return curvature

# Calculate trajectory 
def trajectoryCalc(maxVel, accel):
    timeStamps = []
    angularVel = []
    linearVel = []

    # Telemetry variables (t=spline interval, tim = time in seconds, s = distance travelled, ts = distance travelled in segment, dT = time interval)
    t = 0
    tim = 0
    s = 0
    ts = 0
    dT = 0.010

    # Calculate total distance and segment distances
    totalDistance, segmentDistance = distanceCalc()

    # v = linear velocity, w = angular velocity
    v = 0
    w = 0

    # Segment counter
    tCount = 0

    # Continue calculating until either t is completed or the distance is finished
    while t<len(points)-1 and s<totalDistance:

        # Calculate lienar velocity
        v = linVelGen(s, totalDistance, maxVel)

        # Calculate curvature
        curvature = curvatureCalc(t%1, points[int(t//1)][1], points[int(t//1)][2], points[int(t//1)+1][0], points[int(t//1)+1][1])

        # Avoid dividing by 0
        if curvature == 0:
            w = 0
        else:
            # Calculate angular velocity using radius
            radius = (1/curvature)*12
            w = v/radius

            # Cap the linear velocity based on the angular velocity
            if radius<0:
                if v>(maxLinVel/maxAngularVel)*w+maxLinVel:
                    w = maxLinVel/(radius-(maxLinVel/maxAngularVel))
                    v = w*radius
            else:
                if v>-(maxLinVel/maxAngularVel)*w+maxLinVel:
                    w = maxLinVel/(radius+(maxLinVel/maxAngularVel))
                    v = w*radius

        # Calculate distance change and increment
        s+=(v*dT)/12
        ts+=(v*dT)/12
        
        # Increment time
        tim+=dT

        # Adjust t to ensure correct segment
        if t-tCount>1:
            tCount+=1
            ts = 0
        
        # Calculate t
        t=ts/segmentDistance[int(t//1)]+tCount
        
        # Append values to telemetry lists
        timeStamps.append(tim)
        angularVel.append(w)
        linearVel.append(v)

    # Graph linear and angular vel
    graphVel(timeStamps, linearVel, angularVel)

# Plot angular and linear velocity
def graphVel(t, v, w):
    fig, (ax1,ax2) = plt.subplots(2)
    ax1.plot(t, v)
    ax1.set_title("Linear Velocity")

    ax2.plot(t, w)
    ax2.set_title("Angular Velocity")

    plt.show()

# CONSTANTS
maxAngularVel = 7.363
maxLinVel = 76.576
accel = 1.5
maxVel = 100


WIDTH, HEIGHT = 1020, 720
WIN = pygame.display.set_mode((WIDTH, HEIGHT))




pygame.font.init()
font = pygame.font.Font(None, 36)

run = True

bg = pygame.image.load("Vex Field.png")



splinePoints = []

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
    pygame.draw.lines(WIN, (255, 255, 255), False, splinePoints, width=3)

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