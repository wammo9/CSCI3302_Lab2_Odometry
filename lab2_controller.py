"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor
# import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
GROUND_SENSOR_THRESHOLD = 600

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 0
CENTER_IDX = 1
RIGHT_IDX = 2

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1256
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice(
    'gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10):
    robot.step(SIM_TIMESTEP)

# Initialize variable for left and right speed
vL = 0
vR = 0

# Set constant for startTime
STARTING_TIME = robot.getTime()

# variables for loop closure
TIME_SEEING_LINE_THRESHOLD = 0.1
line_seen = False
last_time_seeing_line = 0


def seing_start_line(gsr):
    return gsr[CENTER_IDX] < GROUND_SENSOR_THRESHOLD and gsr[RIGHT_IDX] < GROUND_SENSOR_THRESHOLD and gsr[LEFT_IDX] < GROUND_SENSOR_THRESHOLD

def update_odometry(vL, vR):
    global pose_x, pose_y, pose_theta
    linear_l = (vL / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED
    linear_r = (vR / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED
    # all this is pretty much just taken from the slides
    v = ((linear_l / 2) + (linear_r / 2))
    omega = (linear_r / EPUCK_AXLE_DIAMETER) - (linear_l / EPUCK_AXLE_DIAMETER)
    xI_dot = math.cos(pose_theta) * v
    yI_dot = math.sin(pose_theta) * v
    theta_dot = omega
    delta_t = SIM_TIMESTEP/1000
    # integrate to get position
    pose_x     += xI_dot * delta_t
    pose_y     += yI_dot * delta_t
    pose_theta += theta_dot * delta_t

state = "speed_measurement"

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()


    # print(gsr)

    match state:
    # Part 1
        case "speed_measurement":
            state = "line_follower"
            # vL = MAX_SPEED
            # vR = MAX_SPEED
            # if seing_start_line(gsr):
            #     vL = 0
            #     vR = 0
            #     print(robot.getTime() - STARTING_TIME)
                # state = "line_follower"

    # Part 2
    # TODO: Implement Line Following under state "line_follower"
    # TODO: Also implement update_odometry and then call update_odometry here
    # Hints for Line Following:
    #
    # 1) Setting vL=MAX_SPEED and vR=-MAX_SPEED lets the robot turn
    # right on the spot. vL=MAX_SPEED and vR=0.5*MAX_SPEED lets the
    # robot drive a right curve.
    #
    # 2) If your robot "overshoots", turn slower.
    #
    # 3) Only set the wheel speeds once so that you can use the speed
    # that you calculated in your odometry calculation.
    #
    # 4) Disable all console output to simulate the robot superfast
    # and test the robustness of your approach.
    #
    # Hints for update_odometry:
    #
    # 1) Divide vL/vR by MAX_SPEED to normalize, then multiply with
    # the robot's maximum speed in meters per second.
    #
    # 2) SIM_TIMESTEP tells you the elapsed time per step. You need
    # to divide by 1000.0 to convert it to seconds
    #
    # 3) Do simple sanity checks. In the beginning, only one value
    # changes. Once you do a right turn, this value should be constant.
    #
    # 4) Focus on getting things generally right first, then worry
    # about calculating odometry in the world coordinate system of the
    # Webots simulator first (x points down, y points right)
        case "line_follower":
            # Center Sensor detects line -> drive forward
            if gsr[CENTER_IDX] < GROUND_SENSOR_THRESHOLD:
                vL = MAX_SPEED
                vR = MAX_SPEED
            # Right Sensor detects line -> rotate clockwise in place
            elif gsr[RIGHT_IDX] < GROUND_SENSOR_THRESHOLD:
                vL = MAX_SPEED
                vR = -MAX_SPEED
            # Left Sensor detects line -> rotate counter-clockwise in place
            elif gsr[LEFT_IDX] < GROUND_SENSOR_THRESHOLD:
                vL = -MAX_SPEED
                vR = MAX_SPEED
            # No sensors detect line -> rotate counter-clockwise in place to reacquire
            else:
                vL = -MAX_SPEED
                vR = MAX_SPEED
            update_odometry(vL, vR)


            if seing_start_line(gsr):
                if line_seen == False:
                    last_time_seeing_line = robot.getTime()
                line_seen = True
                if robot.getTime() - last_time_seeing_line >= TIME_SEEING_LINE_THRESHOLD:
                    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
                    pose_x, pose_y, pose_theta = 0, 0, 0
            else:
                line_seen = False


    # print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
