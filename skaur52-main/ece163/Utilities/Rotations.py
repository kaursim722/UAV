import math
from . import MatrixMath


def dcm2Euler(dcm):

    yaw = math.atan2(dcm[0][1], dcm[0][0])
    roll = math.atan2(dcm[1][2], dcm[2][2])
    pitch = 0
    bounds = dcm[0][2]

    if (bounds <= 1 and bounds >= -1):
        pitch = -math.asin(dcm[0][2])
    else:
        if bounds > 1:
            pitch = -math.asin(1)
        if bounds < -1:
            pitch = -math.asin(-1)

    return [yaw, pitch, roll]


def euler2DCM(yaw, pitch, roll):

    yaw_cos = math.cos(yaw)
    pitch_cos = math.cos(pitch)
    roll_cos = math.cos(roll)
    yaw_sin = math.sin(yaw)
    pitch_sin = math.sin(pitch)
    roll_sin = math.sin(roll)

    row_1 = [(pitch_cos * yaw_cos), (pitch_cos * yaw_sin), -1 * pitch_sin]

    row_2 = [(roll_sin*pitch_sin*yaw_cos) - (roll_cos*yaw_sin), (roll_sin*pitch_sin*yaw_sin) + (roll_cos*yaw_cos),
        (roll_sin*pitch_cos)]

    row_3 = [(roll_cos*pitch_sin*yaw_cos) + (roll_sin*yaw_sin), (roll_cos*pitch_sin*yaw_sin) - (roll_sin*yaw_cos),
        (pitch_cos*roll_cos)]

    dcm = [ row_1, row_2, row_3]
    return dcm

def ned2enu(points):
    """Transforms a matrix from ned to enu"""

    ned = points

    for i in range(len(ned)):
        for j in range(3):
            if j == 1:
                t = ned[i][j]
                ned[i][j] = ned[i][j - 1]
                ned[i][j - 1] = t
            if j == 2:
                if ned[i][j] != 0:
                    ned[i][j] = -1 * ned[i][j]

    return ned

