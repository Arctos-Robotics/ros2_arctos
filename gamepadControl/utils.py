
# --------------------------------------------------------------------CONSTANTS--------------------------------------------------------------------
DURATION = 0.01

NUM_OF_MOTOR = 6

X_MOTOR_ID = 0
Y_MOTOR_ID = 1
Z_MOTOR_ID = 2
A_MOTOR_ID = 3
B_MOTOR_ID = 4
C_MOTOR_ID = 5

# the below is the AXIS LIMIT processed with rawToProcessedAxisValue()

MIN_XAXISMOTOR = -70000
MAX_XAXISMOTOR = 70000

MIN_YAXISMOTOR = -724748
MAX_YAXISMOTOR = 301154

MIN_ZAXISMOTOR = -416906
MAX_ZAXISMOTOR = 196355

MIN_AAXISMOTOR = -75000
MAX_AAXISMOTOR = 75000

MIN_BAXISMOTOR = -111000
MAX_BAXISMOTOR = 111000

MIN_CAXISMOTOR = -220000
MAX_CAXISMOTOR = 220000

# invert array (True if the control direction is "somehow" inverted, False if not)
    # motor 1 (X): request x, go to x
    # motor 2 (Y): request x, go to -x
    # motor 3 (Z): request x, go to x
    # motor 4 (A): request x, go to x
    # motor 5 (B): request x, go to -x
    # motor 6 (C): request x, go to -x
AXIS_INVERTED = [False, True, False, False, True, True]

DIRECTION_INC = 1
DIRECTION_DEC = 0

DON_T_CARE = 0
ZERO = 0
#TODO: replace direction 1 and 0 with this for clear code.

# -------------------------------------------------------------------- COMMON GLOBAL VARIABLES--------------------------------------------------------------------


def rawToProcessedAxisValue(axisEncodedArr: list):
    processedAxis = [0, 0, 0, 0, 0, 0]
    for i in range(len(processedAxis)):
        if i == B_MOTOR_ID:
            processedAxis[i] = (axisEncodedArr[B_MOTOR_ID] - axisEncodedArr[C_MOTOR_ID])/2
        elif i == C_MOTOR_ID:
            processedAxis[i] = (processedAxis[B_MOTOR_ID] + axisEncodedArr[C_MOTOR_ID])
        else:
            processedAxis[i] = axisEncodedArr[i]
    return processedAxis

def processeedAxisValueToRaw(axisProcessedArr: list):
    rawAxis = [0, 0, 0, 0, 0, 0]
    for i in range(len(rawAxis)):
        if i == B_MOTOR_ID:
            rawAxis[B_MOTOR_ID] = (axisProcessedArr[B_MOTOR_ID])
            rawAxis[C_MOTOR_ID] = (-axisProcessedArr[B_MOTOR_ID])
        elif i == C_MOTOR_ID:
            rawAxis[B_MOTOR_ID] = rawAxis[B_MOTOR_ID] + (axisProcessedArr[C_MOTOR_ID])
            rawAxis[C_MOTOR_ID] = rawAxis[C_MOTOR_ID] + (axisProcessedArr[C_MOTOR_ID])
        else:
            rawAxis[i] = axisProcessedArr[i]
    return rawAxis


def angleToProcessedAxis(angleArr):
    """
    angleArr: An array [6] of angle Value
    This function convert an angle to processed Axis, also check for limit. If 
    the converted axis supass the limit, it will be limited with the nearest limit value.
    THIS FUNCTION ONLY MAKE SURE THAT THE JOINTS DOES NOT ROTATE OUT OF IT RANGE, 
    IT DOES NOT HELP CHECKING FOR POSSIBLE COLLISION WITH ENVIRONMENT OR BETWEEN LINKS OF THE ROBOT!
    """
    axisValue = [0, 0, 0, 0, 0, 0]
    motor_limits = {
        X_MOTOR_ID: (MIN_XAXISMOTOR, MAX_XAXISMOTOR, 23093.57097),
        Y_MOTOR_ID: (MIN_YAXISMOTOR, MAX_YAXISMOTOR, 349013.3758),
        Z_MOTOR_ID: (MIN_ZAXISMOTOR, MAX_ZAXISMOTOR, 219570.7125),
        A_MOTOR_ID: (MIN_AAXISMOTOR, MAX_AAXISMOTOR, 23870.1464),
        B_MOTOR_ID: (MIN_BAXISMOTOR, MAX_BAXISMOTOR, 35327.81668),
        C_MOTOR_ID: (MIN_CAXISMOTOR, MAX_CAXISMOTOR, 70019.09612),
    }
    for id, limits in motor_limits.items():
        minLimit, maxLimit, factor = limits
        axisValue[id] = clamp(angleArr[id] * factor, minLimit, maxLimit)
        if axisValue[id] != (angleArr[id] * factor):
            print(f"axis {id} is limited from ({angleArr[id]  * factor}) due to out of limited range ({axisValue[id]})")

    return axisValue

def processedAxisToAngle(processedAxisArr):
    """
    processedAxisArr: An array [6] of processed Axis Value
    This function convert an processedAxis to Angle.
    """
    angleValue = [0, 0, 0, 0, 0, 0]
    motor_limits = {
        X_MOTOR_ID: (MIN_XAXISMOTOR, MAX_XAXISMOTOR, 23093.57097),
        Y_MOTOR_ID: (MIN_YAXISMOTOR, MAX_YAXISMOTOR, 349013.3758),
        Z_MOTOR_ID: (MIN_ZAXISMOTOR, MAX_ZAXISMOTOR, 219570.7125),
        A_MOTOR_ID: (MIN_AAXISMOTOR, MAX_AAXISMOTOR, 23870.1464),
        B_MOTOR_ID: (MIN_BAXISMOTOR, MAX_BAXISMOTOR, 35327.81668),
        C_MOTOR_ID: (MIN_CAXISMOTOR, MAX_CAXISMOTOR, 70019.09612),
    }
    for id, limits in motor_limits.items():
        minLimit, maxLimit, factor = limits
        angleValue[id] = processedAxisArr[id] / factor

    return angleValue

def clamp(value, min_value, max_value):
    """
    This function limit the value between min and max
    """
    return max(min(value, max_value), min_value)

