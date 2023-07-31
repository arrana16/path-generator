import pygame
import numpy as np
import math
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog

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

def distCompute():
    for i in range(len(points)-1):
        for a in range(0, 10000):
            t = a/10000
            x = valueCalculate(t, points[i][1][0], points[i][2][0], points[i+1][0][0], points[i+1][1][0])
            y = valueCalculate(t, points[i][1][1], points[i][2][1], points[i+1][0][1], points[i+1][1][1])
            distPoints.append((x,y))

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
    global distPoints
    totaldistance = 0
    segmentDistance = []
    distPoints = []
    distCompute()
    # Go through the spline points amnd determine the distance travelled
    for i in range(len(distPoints)-1):
        segLength = math.sqrt((distPoints[i][0]-distPoints[i+1][0])**2+(distPoints[i][1]-distPoints[i+1][1])**2)
        totaldistance+=segLength

        # If the segment is completed, add its value to segmentDistance
        if (i+2)%10000==0:
            segmentDistance.append((totaldistance/60))

    # Convert to feet
    totaldistance/=60
    # Go through the segments and make calculate indiivdiual segment length
    for i in range(1, len(segmentDistance)):
        segmentDistance[-i] = segmentDistance[-i]-segmentDistance[-i-1]
    return totaldistance, segmentDistance

# Generate linear velocity
def linVelGen(distance, maxDistance, maxVel):
    initialVel = 0.5
    acceleration = 100

   # Determine the slope and maximum velocity
    slope = (maxLinVel-3)/accel
    maxVel = maxLinVel*(maxVel/100)

    # Determine how far the robot needs to go to reach the maximum velocity
    accelDistance = (maxVel**2-initialVel)/(2*acceleration)/12

    # If the segment is too small, follow a triangular profile
    if accelDistance*2>maxDistance:
        accelDistance = maxDistance/2

    # First part of trapezoid (accelerating)
    if distance<accelDistance:
        velocity = math.sqrt(initialVel**2 + 2*acceleration*(distance*12))
    
    # Middle part of trapepzoid (steady velocity)
    elif accelDistance<distance<maxDistance-accelDistance:
        velocity = maxVel
    
    # End of trapezoid (decelerating)
    else:
        velocity = math.sqrt(initialVel**2 - 2*acceleration*((distance-maxDistance)*12))

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
def trajectoryCalc(maxVel):
    global timeStamps, angularVel, linearVel
    timeStamps = []
    angularVel = []
    linearVel = []

    # Telemetry variables (t=spline interval, tim = time in seconds, s = distance travelled, ts = distance travelled in segment, dT = time interval)
    t = 0
    tim = 0
    s = 0
    ts = 0
    dT = 0.005

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

        w = round(w, 3)
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


print("Enter a name for your path")
name = input()

# CONSTANTS
maxAngularVel = 7.363
maxLinVel = 76.576
accel = 1.5
maxVel = 100

# Data points


# Render window
WIDTH, HEIGHT = 1020, 720
WIN = pygame.display.set_mode((WIDTH, HEIGHT))




pygame.font.init()
font = pygame.font.Font(None, 36)

run = True

bg = pygame.image.load("Vex Field.png")


# Spline points
splinePoints = []

# Points for computing spline distance
distPoints = []

# Variable for which point to mmove
activeCircle = None
coordinateEdit = 0

# Waypoint or Control Point
circleType = ""

# Starting control points and generate a spline
points = [[[150, 300], [50, 300], [150, 300], False], [[200, 50], [300, 50], [200, 50], False]]
newCompute(points[0][1], points[0][0], points[1][0], points[1][1])

# Change x value
xChange = False
setX = ""

# Change y value
yChange = False
setY = ""

# Change velocity
velChange = False
setVel = ""

# Set x, y, velocity and compute buttons
xVal = Button((750, 80), f'x: {round(points[coordinateEdit][1][0]/60,2)}')
yVal = Button((750, 110), f'y: {round((720-points[coordinateEdit][1][1])/60,2)}')
maximumVelocity = Button((750, 200), f"Max Velocity: {maxVel}%")
compute = Button((810, 400),"Compute")
save = Button((790, 600), "Save and exit")

# Run until program is finished
while run:
    WIN.fill("black")
    WIN.blit(bg, (0, 0))
    # Draw handle lines
    pygame.draw.lines(WIN, (255, 255, 255), False, splinePoints, width=3)

    # Render text and buttons
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
    save.render()

    # Draw the circles
    for i in range(len(points)):
        for a in range(3):
            if a == 0 or a == 2:
                pygame.draw.circle(WIN, "white", points[i][a], 5)
            else:
                pygame.draw.circle(WIN, (3, 198, 252), points[i][1], 8)

        pygame.draw.line(WIN, "white", points[i][0], points[i][1], width=2)
        pygame.draw.line(WIN, "white", points[i][2], points[i][1], width=2)

    # Get mouse coordinates
    x = pygame.mouse.get_pos()[0]
    y = pygame.mouse.get_pos()[1]

    # Check the events
    for event in pygame.event.get():
        # Quit
        if event.type == pygame.QUIT:
            run = False

        # Check if any of the points were clicked
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                for i in range(len(points)):
                    for a in range(3):
                        if points[i][a] != None and math.sqrt((points[i][a][0]-x)**2+(points[i][a][1]-y)**2)<10:
                            activeCircle = [i, a]
                            aVal = a
                            iVal = i
                            coordinateEdit = i

        # Drag the button along the mouse
        if event.type == pygame.MOUSEMOTION:
            if activeCircle!=None:
                if aVal == 1:
                    # Move the waypoint and the handles
                    points[iVal][0][0]+=x-points[iVal][1][0]
                    points[iVal][0][1]+=y-points[iVal][1][1]

                    points[iVal][2][0]+=x-points[iVal][1][0]
                    points[iVal][2][1]+=y-points[iVal][1][1]

                    points[iVal][1][0] = x
                    points[iVal][1][1] = y
                    
                    adjacentCompute(iVal, points)
                else:
                    # If there's only one handle set the value of both handles to the same
                    if points[iVal][3] == False:
                        points[iVal][0][0] = x
                        points[iVal][0][1] = y

                        points[iVal][2][0] = x
                        points[iVal][2][1] = y
                    else:
                        # Translate the opposite handles correctly
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

                    # Recompute the spline after the points are moved
                    adjacentCompute(iVal, points)

        # Check if any of the buttons were clicked
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
        
        if compute.click():
            trajectoryCalc(maxVel)

        if event.type == pygame.MOUSEBUTTONUP:
            activeCircle=None

        if save.click():
            path = open(f"paths/{name}.txt", "w")
            path.write(str(len(linearVel))+"\n")
            path.write(str(linearVel)+"\n")
            path.write(str(angularVel)+"\n")
            run = False
        
        # Set the values after coordinates and velocity are set
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

        # Create a new point when n key gets pressed
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_n:
                points.append([[x, y-100], [x,y], [x, y-100], False])
                points[len(points)-2][3] = True

                points[len(points)-2][2][0] = points[len(points)-2][1][0]+(points[len(points)-2][1][0]-points[len(points)-2][0][0])
                points[len(points)-2][2][1] = points[len(points)-2][1][1]+(points[len(points)-2][1][1]-points[len(points)-2][0][1])

                newCompute(points[len(points)-2][1], points[len(points)-2][2], points[len(points)-1][0], points[len(points)-1][1])

    


    pygame.display.flip()