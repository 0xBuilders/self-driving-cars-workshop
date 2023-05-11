"""epuck_detect controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

MAX_SPEED = 6.28
TIME_STEP = 69
DIST_OBJ = 100.0

# create the Robot instance.
robot = Robot()

# initialize devices
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
camera = robot.getDevice('camera')


leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
for i in range(len(psNames)):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
camera.enable(TIME_STEP)


# set up the motor speeds at 10% of the MAX_SPEED.
leftMotor.setVelocity(0.1 * MAX_SPEED)
rightMotor.setVelocity(0.1 * MAX_SPEED)

while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    
    # detect obstacles
    right_obstacle = psValues[0] > DIST_OBJ or psValues[1] > DIST_OBJ or psValues[2] > DIST_OBJ
    left_obstacle = psValues[5] > DIST_OBJ or psValues[6] > DIST_OBJ or psValues[7] > DIST_OBJ

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

# Enter here exit cleanup code.
