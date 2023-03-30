import time  # Import the Time library
from gpiozero import CamJamKitRobot, DistanceSensor  # Import Ultrasonic Library
from gpiozero import CamJamKitRobot, LineSensor # Import Infrared Library

# Define GPIO pins to use for the infrared/ultrasoinic sensor
pintrigger = 17
pinecho = 18
pinLineFollower = 25

# Define robot and sensors 
robot = CamJamKitRobot()
linesensor = LineSensor(pinLineFollower)
sensor = DistanceSensor(pinecho, pintrigger)

# Ultrasonic Variables 
hownear = 15.0 # How close the robot can get to an object
reversetime = 0.5 # How long (time) robot will reverse after detection
turntime = 0.75 # How long (time) robot will rotate after reversing

# Infrared Variables
direction = True  # The direction the robot will turn - True = Left
isoverblack = True  # A flag to say the robot can see a black line
linelost = False  # A flag that is set if the line has been lost

# Set the relative speeds of the two motors (0.0 - 1.0)
leftmotorspeed = 0.5
rightmotorspeed = 0.5

# Define directions of robot
motorforward = (leftmotorspeed, rightmotorspeed)
motorbackward = (-leftmotorspeed, -rightmotorspeed)
motorleft = (leftmotorspeed, 0)
motorright = (0, rightmotorspeed)

# Fuction for when line is seen
def lineseen():
    global isoverblack, linelost
    print("The line has been found.")
    isoverblack = True
    linelost = False
    robot.value = motorforward

# Function for when line is not seen
def linenotseen():
    global isoverblack
    print("The line has been lost.")
    isoverblack = False

# Function to return true if the ultrasonics detects an obstacle
def isnearobstacle(localhownear):
    distance = sensor.distance * 100

    print("IsNearObstacle: " + str(distance))
    if distance < localhownear:
        return True
    else:
        return False

# Function to search for black line
def findpath():
    global direction, linelost
    robot.stop()

    print("Seeking new path")

    seeksize = 0.25  # Turn for 0.25s
    seekcount = 1  # A count of times the robot has looked for the line
    maxseekcount = 8  # The maximum # of times needed to seek (360 degrees)

    # Turn the robot left then right until it finds the line
    # Or we have looked long enough
    while seekcount <= maxseekcount:
        # Set the seek time
        seektime = seeksize * seekcount

        # Start the motors turning in a direction
        if direction:
            print("Looking left")
            robot.value = motorleft
        else:
            print("Looking Right")
            robot.value = motorright

        # Save the time it is now
        starttime = time.time()

        # While the robot is turning for seektime seconds,
        # check to see whether the line detector is over black
        while (time.time() - starttime) <= seektime:
            if isoverblack: # If a line has been seen that is not in the direction of an obstacle
                robot.value = motorforward
                # Exit the seekline() function returning
                return True

        robot.stop() # The robot has not found the black line yet, so stop
        seekcount += 1 # Increase the seek count
        direction = not direction # Change direction

    # The line wasn't found, so return False
    robot.stop()
    print("The line has been lost - relocate your robot")
    linelost = True
    return False

# Tell the program what to do with a line is seen / not seen
# Call on the seen / not seen functions
linesensor.when_line = lineseen
linesensor.when_no_line = linenotseen

# Function to avoid obstacle 
def avoidobstacle():
    # Back off a little
    print("Backwards")
    robot.value = motorbackward
    time.sleep(reversetime)
    robot.stop()

    # Turn right
    print("Right")
    robot.value = motorright
    time.sleep(turntime)
    robot.stop()

# Main iterative loop
try:
    while True:
        robot.value = motorforward
        time.sleep(0.1) # Check every 0.1s if obstacle is near or line is lost
        if isnearobstacle(hownear): 
            avoidobstacle()
        if not isoverblack and not linelost:
            findpath()

# If you press CTRL+C, cleanup and stop
except KeyboardInterrupt:
    robot.stop()