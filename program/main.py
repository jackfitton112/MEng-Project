from machine import Pin
from math import tan, radians

# Pin Definitions
STEP_A = Pin(0, Pin.OUT)
DIR_A = Pin(1, Pin.OUT)

STEP_B = Pin(2, Pin.OUT)
DIR_B = Pin(3, Pin.OUT)

STEP_C = Pin(4, Pin.OUT)
DIR_C = Pin(5, Pin.OUT)

# Constants for mechanics
MICROSTEP_FACTOR = 32
STEPS_PER_REV = 200 * MICROSTEP_FACTOR

PULLEY_TEETH = 20
BELT_PITCH_MM = 2

MM_PER_STEP = (BELT_PITCH_MM * PULLEY_TEETH) / STEPS_PER_REV
STEPS_PER_MM = STEPS_PER_REV / (BELT_PITCH_MM * PULLEY_TEETH)

# Stepper Class
class Stepper:
    def __init__(self, step_pin, dir_pin, to_motor_direction, current_pos_mm=0):
        self.stepPin = step_pin
        self.dirPin = dir_pin
        self.toMotorDirection = to_motor_direction
        self.currentPos = current_pos_mm
        self.currentDir = to_motor_direction
        self.dirPin.value(to_motor_direction)

    def step(self):
        self.stepPin.value(1)
        self.stepPin.value(0)
        self.currentPos += MM_PER_STEP if self.currentDir else -MM_PER_STEP

    def setDir(self, direction):
        self.dirPin.value(self.toMotorDirection if direction == 0 else not self.toMotorDirection)
        self.currentDir = direction

# Initialize Steppers
StepperA = Stepper(STEP_A, DIR_A, 1, 30)
StepperB = Stepper(STEP_B, DIR_B, 0, 80)
StepperC = Stepper(STEP_C, DIR_C, 0, 150)

# Safety checks for platform positions
def validatePositions(A_mm, B_mm, C_mm):
    positions = [A_mm, B_mm, C_mm]
    # Check absolute position limits
    for pos in positions:
        if pos < 25 or pos > 365:
            print("Error: Position out of bounds. Movement aborted.")
            return False
    # Check minimum separation of platforms
    if abs(A_mm - B_mm) < 40 or abs(B_mm - C_mm) < 40 or abs(A_mm - C_mm) < 40:
        print("Error: Platforms too close. Movement aborted.")
        return False
    return True

# Set absolute positions of motors
def setMotorPos(A_mm, B_mm, C_mm):


    deltaA_mm = A_mm - StepperA.currentPos
    deltaB_mm = B_mm - StepperB.currentPos
    deltaC_mm = C_mm - StepperC.currentPos

    StepperA.setDir(0 if deltaA_mm < 0 else 1)
    StepperB.setDir(0 if deltaB_mm < 0 else 1)
    StepperC.setDir(0 if deltaC_mm < 0 else 1)

    stepsA = int(abs(deltaA_mm) * STEPS_PER_MM)
    stepsB = int(abs(deltaB_mm) * STEPS_PER_MM)
    stepsC = int(abs(deltaC_mm) * STEPS_PER_MM)

    maxSteps = max(stepsA, stepsB, stepsC)

    for step in range(maxSteps):
        if step < stepsA:
            StepperA.step()
        if step < stepsB:
            StepperB.step()
        if step < stepsC:
            StepperC.step()

# Inverse Kinematics for Tripteron Robot
def inverseKinematics(x, y, z):
    tan60 = tan(radians(60))
    tan50 = tan(radians(50))

    delta_A = -x - (y / tan60)
    delta_C = -x + (y / tan60)
    delta_B = -z - (x / tan50)

    print("Kinematics:", delta_A, delta_B, delta_C) 

    return delta_A, delta_B, delta_C

# Set End Effector Position
def moveTo(x, y, z):
    A_mm, B_mm, C_mm = inverseKinematics(x, y, z)
    setMotorPos(A_mm, B_mm, C_mm)

# Example main usage
def main():
    while True:
        setMotorPos(50, 100, 150)
        sleep(1)
        setMotorPos(100, 200, 250)
        sleep(1)
        setMotorPos(150, 250, 300)
        sleep(1)
    #print(f"Positions (A, B, C): {StepperA.currentPos:.2f}mm, {StepperB.currentPos:.2f}mm, {StepperC.currentPos:.2f}mm")

if __name__ == "__main__":
    main()

