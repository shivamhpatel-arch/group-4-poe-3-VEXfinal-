# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       g4-p3-VEXtransport .py                                                      #
# 	Author:       Shivam, Paxton & Eric                                           #
# 	Created:      6/2/2026, 10:39:49 AM                                        #
# 	Description:  Code for POE RECbot final                                 #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

# ------------------------------------------ Robot Configuration Code ---------------------------------------------------------------------
rightMotor = Motor(Ports.PORT7, GearSetting.RATIO_18_1, False) #Right DriveTrain Motor
leftMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)   #Left DriveTrain Motor
drivetrain = DriveTrain(leftMotor, rightMotor)                 #Start and stop the motors simultaneously
# Set the leftMotor reverse properly to True so that when driving forward
# Same direction as the right motor
liftMotor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False) #Liftarm motor
inertial_1 = Inertial(Ports.PORT5)                            #Inertial Sensor
LiftArmRotation = Rotation(Ports.PORT6, False)                #Liftarm rotations
bumpSwitch = Bumper(brain.three_wire_port.a)                  #Bumper Switch
# -----------------------------------------------------------------------------------------------------------------------------------------

# ------------------------------------------ Helper Functions -----------------------------------------------------------------------------
def bump():
    """
    Hold the program's execution until the button is pressed
    """

    while bumpSwitch.pressing() == False:
        wait(10, MSEC) #Debounce the button

        brain.screen.set_cursor(1, 1)
        brain.screen.print("Press the button to start the program")
        pass

    brain.screen.clear_line(1)
    brain.screen.set_cursor(1,1)
    brain.screen.print("Program executed")
    wait(1, SECONDS)

def inertialCalibration():
    """
    1. Calibrate the inertial
    2. A wait time of 2 seconds is required to complete the calibration
    3. This function is called at the start of the program's execution
    """

    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("Calibration in process")
    brain.screen.set_cursor(2, 1)
    brain.screen.print("Don't move the robot")
    inertial_1.calibrate() #Calibrate the inertial sensor

    wait(2, SECONDS)       #Time required to perform calibration

    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("Calibration complete")

def testInertial():
    """
    1. Test the inertial sensor by having it display heading and rotation data.
    2. Pressing the bumper switch will end the text
    """
    brain.screen.clear_screen()
    while (bumpSwitch.pressing() == False):
        brain.screen.set_cursor(5, 1)
        brain.screen.print("Heading:  " + str(inertial_1.heading()))
        brain.screen.set_cursor(6, 1)
        brain.screen.print("Rotation:  " + str(inertial_1.rotation()))
        brain.screen.set_cursor(8, 1)
        brain.screen.print("Press the bump switch to exit")
        
    brain.screen.clear_screen()
    brain.screen.set_cursor(8, 1)
    brain.screen.print("Inertial test complete")

def driveStraightData(e):
    """
    1. Report the position, rotation, and error values to the screen while driving
    2. Parameter: e = error value
    """

    brain.screen.set_cursor(1, 1)
    brain.screen.print("Position: " + str(leftMotor.position()))

    brain.screen.set_cursor(2, 1)
    brain.screen.print("Rotation: " + str(inertial_1.rotation()))

    brain.screen.set_cursor(3, 1)
    brain.screen.print("Error: " + str(e))

def stopMotors():
    drivetrain.stop()
    wait(0.5, SECONDS) #Wait 0.5 seconds for stability before performing next action

def driveStraight(distance, setpoint, motorVelocity):
    """
    1. distance = distance to travel in inches
    2. setpoint = 0-degrees of rotation for driving straight
    3. motorVelocity = the velocity for motor (+) => forward and (-) => Reverse
    """

    inertial_1.reset_rotation()

    leftMotor.set_stopping(COAST)
    rightMotor.set_stopping(COAST)

    kP = 0.45          #Proportional constant for driving straight
                      #Used to calculate the correction term to maintain course
                      #If the value is too small, correction will occur to slowly
   
    wheelDiameter = 4

    #Calculate the distance we want to travel in terms of encoder ticks
    #Distance (ticks) = (Distance in inches)

    wheelCircumference = wheelDiameter * math.pi
    distance = (distance / wheelCircumference) * 360 #Distance in ticks

    #Reset the motor encoder count to zero before driving
    leftMotor.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)

    #Drive forward if the motor velocity is greater than zero

    if(motorVelocity > 0):
        while(leftMotor.position() < distance):
            error = (setpoint - inertial_1.rotation())
            correction = error * kP

            #if error > 0 (setpoint > rotation) => robot is drifting left
            #if error < 0 => robot is drifting to the right
            leftMotor.set_velocity((motorVelocity + correction), PERCENT)
            rightMotor.set_velocity((motorVelocity - correction), PERCENT)

            drivetrain.drive(FORWARD)

            driveStraightData(error)

        stopMotors()

    else:

        distance *= -1 #Driving backward requires a negative encoder count value
        while(leftMotor.position() > distance):
            error = (setpoint - inertial_1.rotation())
            correction = error * kP

            #if error > 0 (setpoint > rotation) => robot is drifting left
            #if error < 0 => robot is drifting to the right
            leftMotor.set_velocity((motorVelocity + correction), PERCENT)
            rightMotor.set_velocity((motorVelocity - correction), PERCENT)

            drivetrain.drive(FORWARD)

            driveStraightData(error)

        stopMotors()

def turnData(turnError, derivative):
    """
    Print the current heading, error and derivatives for the point turn
    """

    brain.screen.set_cursor(1, 1)
    brain.screen.print("Heading: " + str(inertial_1.heading())) #Return current heading

    brain.screen.set_cursor(2, 1)
    brain.screen.print("Error: " + str(abs(turnError)))         #Return current turnError

    brain.screen.set_cursor(3, 1)
    brain.screen.print("Derivative: " + str(abs(derivative)))   #Return current derivative

def pointTurn(setPoint):
    """
    1. Perform a point turn using the inertial sensor and its heading
    2. Proportional and derivative control are required to maintain accuracy
    """

    brain.screen.clear_screen()      #Clear the screen in preparation for the data
    
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    #Calculate the difference between the setpoint and the current heading
    difference = setPoint - inertial_1.heading()

    #Want to turn the smallest amount to reach the set point
    if (setPoint > inertial_1.heading()):
        if (abs(difference) <= 180):
            clockwise = True
        else:
            clockwise = False
    else:
        if (abs(difference) <= 180):
            clockwise = False
        else:
            clockwise = True

    #Define kP and kD for clockwise and counterclockwise turns

    if(clockwise):
        kP = 0.0856
        kD = 0.0415
    else:
        kP = 0.08535
        kD = 0.0

    maxVelocity = 50    #Units: %
    previousError = 0   #Error from the previous iteration 

    while(True):
        turnError = setPoint - inertial_1.heading()
        derivative = turnError - previousError

        #Stop the motor and exit the loop if the magnitude of the error and derivative terms are small

        if ((abs(turnError) < 1) and (abs(derivative) < 0.2)):
            stopMotors() #Stop the motors
            break        #Exit the while loop

        #This term will be larger than one depending upon the amount of turn required
        turnCorrection = (kP * turnError) + (kD * derivative)

        if (abs(turnCorrection) > 1):
            turnCorrection = 1

        turnVelocity = turnCorrection * maxVelocity #Calculate the new motor velocity

        #Set motor velocity based on the direction and PD output

        if(clockwise):
            leftMotor.set_velocity(turnVelocity)
            rightMotor.set_velocity(-1 * turnVelocity)
        else:
            leftMotor.set_velocity(-1 * turnVelocity)
            rightMotor.set_velocity(turnVelocity)

        leftMotor.spin(FORWARD)
        rightMotor.spin(FORWARD)

        turnData(turnError, derivative)

        previousError = turnError

        wait(20, MSEC)

def liftArm(motorVelocity, liftAngle):

    #Config the motor to hold it's position
    liftMotor.set_stopping(HOLD)
    
    liftMotor.set_velocity(motorVelocity, PERCENT) #Set motor lift velocity

    gearRatio = 5 #60T to 12T
    motorAngularDisplacement = liftAngle * gearRatio

    liftMotor.spin_for(FORWARD, motorAngularDisplacement)
# ------------------------------------------ Define the main() function -------------------------------------------------------------------
def main():
    """
    The main() function is the program that will be executed by the VEX brain
    """

    bump()                #Call the bump() function to begin program execution
    inertialCalibration() #Calibrate the inertial sensor

    driveStraight(90, 0, 75)
    liftArm(20, 45)
    wait(1, SECONDS)
    driveStraight(11 , 0, -50)
    pointTurn(90)
    driveStraight(65, 0, 50)
    pointTurn(50)
    driveStraight(15, 0, 50)
    liftArm(20, -45)
    liftArm(20, 90)
    driveStraight(2, 0, -50)
    pointTurn(150)
    driveStraight(8, 0, -75)
    pointTurn(90)
    driveStraight(30, 0, -75)

# ------------------------------------------ Call the main() function -------------------------------------------------------------------

main()
