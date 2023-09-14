import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations


def computeGains(tuningParameters=Controls.controlTuning(), linearizedModel=Linearized.transferFunctions()):
    control_G = Controls.controlGains()

    wnroll = tuningParameters.Wn_roll
    wnpitch = tuningParameters.Wn_pitch
    wn_alt = tuningParameters.Wn_altitude
    wnSide = tuningParameters.Wn_sideslip
    wnCourse = tuningParameters.Wn_course
    wn_speedT = tuningParameters.Wn_SpeedfromThrottle
    wn_speedE = tuningParameters.Wn_SpeedfromElevator

    zroll = tuningParameters.Zeta_roll
    zpitch = tuningParameters.Zeta_pitch
    zalt = tuningParameters.Zeta_altitude
    zcourse = tuningParameters.Zeta_course
    zslip = tuningParameters.Zeta_sideslip
    ze = tuningParameters.Zeta_SpeedfromElevator
    zt = tuningParameters.Zeta_SpeedfromThrottle

    kiroll = 0.001
    kproll = (wnroll ** 2.0) / linearizedModel.a_phi2
    kdroll = ((2.0 * zroll * wnroll) - linearizedModel.a_phi1)/linearizedModel.a_phi2
    ki_course = ((wnCourse ** 2.0) * linearizedModel.Va_trim) / VPC.g0
    kp_course = (2.0 * zcourse * wnCourse * linearizedModel.Va_trim) / VPC.g0
    kp_pitch = (wnpitch ** 2.0 - linearizedModel.a_theta2) / linearizedModel.a_theta3
    kd_pitch = (2.0 * zpitch*wnpitch - linearizedModel.a_theta1) / linearizedModel.a_theta3
    ki_side = (wnSide ** 2.0)/linearizedModel.a_beta2
    kp_side = ((2.0 * zslip * wnSide) - linearizedModel.a_beta1)/ linearizedModel.a_beta2
    k_theta = (kp_pitch * linearizedModel.a_theta3) / (linearizedModel.a_theta2 + (kp_pitch * linearizedModel.a_theta3))
    ki_alt = wn_alt ** 2.0 /(k_theta * linearizedModel.Va_trim)
    kp_alt = (2.0 * zalt * wn_alt) / (k_theta * linearizedModel.Va_trim)
    ki_T = wn_speedT ** 2.0 / linearizedModel.a_V2
    kp_T = ((2.0 * zt * wn_speedT) - linearizedModel.a_V1) / linearizedModel.a_V2
    ki_E = (-wn_speedE ** 2.0) / (k_theta * VPC.g0)
    kp_E = (linearizedModel.a_V1 - (2.0 * ze * wn_speedE)) / (k_theta * VPC.g0)

    control_G.kp_roll = kproll
    control_G.kd_roll = kdroll
    control_G.ki_roll = kiroll
    control_G.ki_course = ki_course
    control_G.kp_course = kp_course
    control_G.kp_pitch = kp_pitch
    control_G.kd_pitch = kd_pitch
    control_G.ki_sideslip = ki_side
    control_G.kp_sideslip = kp_side
    control_G.kp_altitude = kp_alt
    control_G.ki_altitude = ki_alt
    control_G.kp_SpeedfromElevator = kp_E
    control_G.ki_SpeedfromElevator = ki_E
    control_G.ki_SpeedfromThrottle = ki_T
    control_G.kp_SpeedfromThrottle = kp_T


    return control_G

def computeTuningParameters(controlGains=Controls.controlGains(), linearizedModel=Linearized.transferFunctions()):
    tuningParam = Controls.controlTuning()

    k_theta = (controlGains.kp_pitch * linearizedModel.a_theta3) / (linearizedModel.a_theta2 + (controlGains.kp_pitch * linearizedModel.a_theta3))

    tuningParam.Wn_roll = math.sqrt(controlGains.kp_roll * linearizedModel.a_phi2)
    tuningParam.Wn_course = math.sqrt((controlGains.ki_course * VPC.g0) / linearizedModel.Va_trim)
    tuningParam.Wn_pitch = math.sqrt(controlGains.kp_pitch * linearizedModel.a_theta3 + linearizedModel.a_theta2)
    tuningParam.Wn_altitude = math.sqrt(k_theta * linearizedModel.Va_trim * controlGains.ki_altitude)
    tuningParam.Wn_sideslip = math.sqrt(controlGains.ki_sideslip * linearizedModel.a_beta2)
    tuningParam.Wn_SpeedfromElevator = math.sqrt(-k_theta * VPC.g0 * controlGains.ki_SpeedfromElevator)
    tuningParam.Wn_SpeedfromThrottle = math.sqrt(linearizedModel.a_V2 * controlGains.ki_SpeedfromThrottle)


    tuningParam.Zeta_roll = (linearizedModel.a_phi1 + (linearizedModel.a_phi2 * controlGains.kd_roll)) / (2.0 * tuningParam.Wn_roll)
    tuningParam.Zeta_course = ((controlGains.kp_course * VPC.g0) / linearizedModel.Va_trim) / (2.0 * tuningParam.Wn_course)
    tuningParam.Zeta_pitch = (linearizedModel.a_theta1 + (controlGains.kd_pitch * linearizedModel.a_theta3)) / (2.0 * tuningParam.Wn_pitch)
    tuningParam.Zeta_altitude = (k_theta * linearizedModel.Va_trim * controlGains.kp_altitude) / (2.0 * tuningParam.Wn_altitude)
    tuningParam.Zeta_sideslip = (linearizedModel.a_beta1 + (linearizedModel.a_beta2 * controlGains.kp_sideslip)) / (2.0 * tuningParam.Wn_sideslip)
    tuningParam.Zeta_SpeedfromElevator = (linearizedModel.a_V1 - (k_theta * VPC.g0 * controlGains.kp_SpeedfromElevator)) / (2.0 * tuningParam.Wn_SpeedfromElevator)
    tuningParam.Zeta_SpeedfromThrottle = (linearizedModel.a_V1 + (linearizedModel.a_V2 * controlGains.kp_SpeedfromThrottle)) / (2.0 * tuningParam.Wn_SpeedfromThrottle)


    return tuningParam