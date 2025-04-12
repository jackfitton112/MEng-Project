from machine import Pin, UART
from math import tan, radians
import time

# UART Pin Definitions
TX = Pin(11, Pin.OUT)
RX = Pin(12, Pin.IN)


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
        self.startPos = current_pos_mm

    def step(self):
        self.stepPin.value(1)
        self.stepPin.value(0)
        self.currentPos += MM_PER_STEP if self.currentDir else -MM_PER_STEP

    def setDir(self, direction):
        self.dirPin.value(self.toMotorDirection if direction == 0 else not self.toMotorDirection)
        self.currentDir = direction

# Initialize Steppers
StepperA = Stepper(STEP_A, DIR_A, 1, 35)
StepperB = Stepper(STEP_B, DIR_B, 0, 85)
StepperC = Stepper(STEP_C, DIR_C, 0, 230)

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
    # Calculate step deltas
    deltaA_mm = A_mm - StepperA.currentPos
    deltaB_mm = B_mm - StepperB.currentPos
    deltaC_mm = C_mm - StepperC.currentPos

    # Set direction
    StepperA.setDir(0 if deltaA_mm < 0 else 1)
    StepperB.setDir(0 if deltaB_mm < 0 else 1)
    StepperC.setDir(0 if deltaC_mm < 0 else 1)

    # Calculate absolute steps
    stepsA = abs(deltaA_mm) * STEPS_PER_MM
    stepsB = abs(deltaB_mm) * STEPS_PER_MM
    stepsC = abs(deltaC_mm) * STEPS_PER_MM

    # Round steps to integers
    stepsA = int(round(stepsA))
    stepsB = int(round(stepsB))
    stepsC = int(round(stepsC))

    # Determine the maximum steps (longest axis movement)
    maxSteps = max(stepsA, stepsB, stepsC)

    # Initialize accumulators
    accA = accB = accC = 0

    for step in range(maxSteps):
        accA += stepsA
        accB += stepsB
        accC += stepsC

        if accA >= maxSteps:
            StepperA.step()
            accA -= maxSteps

        if accB >= maxSteps:
            StepperB.step()
            accB -= maxSteps

        if accC >= maxSteps:
            StepperC.step()
            accC -= maxSteps

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

#rectangular path
commands = [(74, 198, 248), (74, 198, 248), (74, 198, 248), (74, 198, 248), (74, 198, 248), (75, 198, 248), (75, 198, 248), (75, 198, 248), (76, 198, 248), (76, 197, 247), (76, 197, 247), (77, 197, 247), (77, 197, 247), (77, 197, 247), (78, 197, 247), (78, 197, 247), (78, 197, 247), (79, 197, 247), (79, 197, 247), (79, 197, 247), (80, 197, 247), (80, 197, 247), (80, 197, 247), (81, 197, 247), (81, 197, 247), (81, 197, 247), (82, 197, 247), (82, 197, 247), (83, 197, 247), (83, 197, 247), (83, 197, 247), (84, 197, 247), (84, 198, 248), (85, 198, 248), (85, 198, 248), (85, 199, 249), (86, 199, 249), (86, 199, 249), (86, 199, 249), (87, 200, 250), (87, 200, 250), (87, 200, 250), (88, 201, 251), (88, 201, 251), (89, 201, 251), (89, 202, 252), (89, 202, 252), (89, 203, 253), (90, 203, 253), (90, 203, 253), (90, 204, 254), (90, 204, 254), (90, 205, 255), (90, 205, 255), (90, 205, 255), (90, 206, 256), (90, 206, 256), (91, 207, 257), (91, 207, 257), (91, 207, 257), (91, 208, 258), (91, 208, 258), (91, 208, 258), (92, 209, 259), (92, 209, 259), (92, 209, 259), (92, 210, 260), (92, 210, 260), (92, 211, 261), (92, 211, 261), (92, 211, 261), (92, 212, 262), (91, 212, 262), (91, 212, 262), (91, 212, 262), (91, 212, 262), (91, 213, 263), (91, 214, 264), (90, 214, 264), (90, 214, 264), (91, 214, 264), (90, 214, 264), (89, 215, 265), (90, 215, 265), (90, 215, 265), (89, 215, 265), (89, 215, 265), (89, 215, 265), (88, 216, 266), (89, 216, 266), (89, 216, 266), (88, 216, 266), (88, 216, 266), (88, 217, 267), (88, 217, 267), (87, 217, 267), (87, 217, 267), (87, 217, 267), (87, 217, 267), (87, 218, 268), (87, 218, 268), (86, 218, 268), (86, 218, 268), (86, 218, 268), (86, 218, 268), (86, 219, 269), (85, 219, 269), (86, 219, 269), (85, 219, 269), (85, 219, 269), (85, 220, 270), (84, 220, 270), (84, 220, 270), (85, 220, 270), (84, 220, 270), (84, 221, 271), (84, 220, 270), (84, 221, 271), (83, 221, 271), (83, 221, 271), (83, 221, 271), (83, 222, 272), (83, 221, 271), (83, 222, 272), (82, 222, 272), (82, 222, 272), (82, 223, 273), (82, 222, 272), (81, 223, 273), (82, 223, 273), (81, 223, 273), (81, 223, 273), (81, 224, 274), (81, 223, 273), (80, 224, 274), (81, 224, 274), (80, 224, 274), (80, 224, 274), (80, 225, 275), (79, 225, 275), (80, 225, 275), (79, 225, 275), (79, 225, 275), (79, 225, 275), (79, 226, 276), (78, 226, 276), (79, 226, 276), (78, 226, 276), (78, 226, 276), (78, 226, 276), (78, 227, 277), (78, 227, 277), (77, 226, 276), (78, 226, 276), (78, 227, 277), (79, 228, 278), (80, 229, 279), (80, 229, 279), (81, 230, 280), (81, 231, 281), (82, 232, 282), (82, 232, 282), (82, 233, 283), (82, 234, 284), (83, 235, 285), (83, 236, 286), (83, 236, 286), (83, 237, 287), (82, 238, 288), (82, 238, 288), (82, 239, 289), (82, 239, 289), (81, 240, 290), (81, 240, 290), (80, 240, 290), (80, 241, 291), (79, 241, 291), (78, 241, 291), (78, 241, 291), (77, 241, 291), (76, 241, 291), (76, 241, 291), (75, 241, 291), (74, 240, 290), (73, 240, 290), (72, 240, 290), (72, 239, 289), (71, 239, 289), (70, 238, 288), (69, 238, 288), (69, 237, 287), (68, 236, 286), (67, 236, 286), (67, 235, 285), (66, 234, 284), (66, 233, 283), (65, 232, 282), (65, 232, 282), (65, 231, 281), (64, 230, 280), (64, 229, 279), (64, 229, 279), (64, 228, 278), (64, 227, 277), (64, 226, 276), (64, 226, 276), (63, 225, 275), (63, 225, 275), (62, 225, 275), (61, 225, 275), (61, 225, 275), (60, 225, 275), (59, 225, 275), (58, 224, 274), (57, 224, 274), (57, 224, 274), (56, 223, 273), (55, 223, 273), (54, 222, 272), (54, 222, 272), (53, 221, 271), (52, 220, 270), (52, 220, 270), (51, 219, 269), (50, 218, 268), (50, 217, 267), (49, 216, 266), (49, 216, 266), (49, 215, 265), (48, 214, 264), (48, 213, 263), (48, 213, 263), (48, 212, 262), (48, 211, 261), (48, 210, 260), (48, 210, 260), (48, 209, 259), (49, 209, 259), (49, 208, 258), (49, 208, 258), (49, 208, 258), (50, 207, 257), (50, 207, 257), (51, 207, 257), (51, 207, 257), (52, 206, 256), (53, 206, 256), (54, 206, 256), (54, 206, 256), (55, 207, 257), (56, 207, 257), (57, 207, 257), (57, 207, 257), (58, 208, 258), (59, 208, 258), (60, 209, 259), (61, 209, 259), (61, 210, 260), (62, 211, 261), (62, 210, 260), (62, 210, 260), (62, 210, 260), (63, 211, 261), (63, 212, 262), (64, 213, 263), (64, 213, 263), (65, 214, 264), (65, 215, 265), (66, 216, 266), (66, 217, 267), (66, 217, 267), (67, 218, 268), (67, 219, 269), (67, 220, 270), (67, 220, 270), (67, 221, 271), (67, 222, 272), (66, 222, 272), (66, 223, 273), (66, 223, 273), (65, 224, 274), (65, 224, 274), (64, 224, 274), (64, 225, 275), (64, 225, 275), (64, 225, 275), (65, 224, 274), (65, 224, 274), (65, 224, 274), (66, 223, 273), (66, 223, 273), (67, 223, 273), (67, 222, 272), (68, 222, 272), (69, 222, 272), (69, 222, 272), (70, 222, 272), (71, 223, 273), (72, 223, 273), (72, 223, 273), (73, 223, 273), (74, 224, 274), (75, 224, 274), (76, 225, 275), (76, 225, 275), (76, 225, 275), (62, 210, 260), (62, 210, 260), (62, 210, 260), (63, 210, 260), (63, 210, 260), (63, 209, 259), (63, 209, 259), (63, 209, 259), (63, 209, 259), (64, 209, 259), (64, 209, 259), (64, 208, 258), (64, 208, 258), (64, 208, 258), (64, 208, 258), (65, 208, 258), (65, 208, 258), (65, 207, 257), (65, 207, 257), (65, 207, 257), (65, 207, 257), (66, 207, 257), (66, 207, 257), (66, 206, 256), (66, 206, 256), (66, 206, 256), (66, 206, 256), (67, 206, 256), (67, 206, 256), (67, 205, 255), (67, 205, 255), (67, 205, 255), (67, 205, 255), (68, 205, 255), (68, 205, 255), (68, 204, 254), (68, 204, 254), (68, 204, 254), (68, 204, 254), (69, 204, 254), (69, 204, 254), (69, 203, 253), (69, 203, 253), (69, 203, 253), (69, 203, 253), (70, 203, 253), (70, 203, 253), (70, 202, 252), (70, 202, 252), (70, 202, 252), (70, 202, 252), (71, 202, 252), (71, 202, 252), (71, 201, 251), (71, 201, 251), (71, 201, 251), (71, 201, 251), (72, 201, 251), (72, 201, 251), (72, 200, 250), (72, 200, 250), (72, 200, 250), (72, 200, 250), (73, 200, 250), (73, 200, 250), (73, 199, 249), (73, 199, 249), (73, 199, 249), (73, 199, 249), (74, 199, 249), (74, 199, 249)]
def run():
    for command in commands:
        A, B, C = command
        setMotorPos(A,B,C)
        time.sleep(0.1)

    #return to home position
    setMotorPos(StepperA.startPos, StepperB.startPos, StepperC.startPos)




