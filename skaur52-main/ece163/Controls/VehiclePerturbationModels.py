import math
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath


def CreateTransferFunction(trimState, trimInputs):

    alpha_trim = trimState.alpha
    theta_trim = trimState.pitch
    gamma_trim = theta_trim - alpha_trim
    phi_trim = trimState.roll
    chi_trim = trimState.chi
    trimState.Va = math.hypot(trimState.u, trimState.v, trimState.w)
    Throttle = trimInputs.Throttle
    Elevator = trimInputs.Elevator

    if math.isclose(trimState.Va, 0.0):
        beta_trim = 0.0
    else:
        beta_trim = trimState.beta

    a_beta1 = ((-VPC.rho * (trimState.Va) * VPC.S) / (2.0 * VPC.mass)) * VPC.CYbeta
    a_beta2 = ((VPC.rho * (trimState.Va) * VPC.S) / (2.0 * VPC.mass)) * VPC.CYdeltaR
    a_phi1 = -0.5 * VPC.rho * (trimState.Va ** 2.0) * VPC.S * VPC.b * VPC.Cpp * (VPC.b / (2.0 * trimState.Va))
    a_phi2 = 0.5 * VPC.rho * (trimState.Va ** 2.0) * VPC.S * VPC.b * VPC.CpdeltaA
    a_theta1 = ((-VPC.rho * (trimState.Va ** 2.0) * VPC.c * VPC.S) / (2.0 * VPC.Jyy)) * VPC.CMq * (VPC.c / (2.0 * trimState.Va))
    a_theta2 = ((-VPC.rho * (trimState.Va ** 2.0) * VPC.c * VPC.S) / (2.0 * VPC.Jyy)) * VPC.CMalpha
    a_theta3 = ((VPC.rho * (trimState.Va ** 2.0) * VPC.c * VPC.S) / (2.0 * VPC.Jyy)) * VPC.CMdeltaE

    a_V1 = (((VPC.rho * trimState.Va * VPC.S) / VPC.mass) * (VPC.CD0 + (VPC.CDalpha * alpha_trim) +
            (VPC.CDdeltaE * Elevator))) - ((1.0 / VPC.mass) * dThrust_dVa(trimState.Va, Throttle))
    a_V2 = (1.0 / VPC.mass) * dThrust_dThrottle(trimState.Va, Throttle)
    a_V3 = VPC.g0 * math.cos(theta_trim - alpha_trim)

    return Linearized.transferFunctions(Va_trim= trimState.Va, alpha_trim= alpha_trim, beta_trim= beta_trim,
                                         gamma_trim= gamma_trim, theta_trim= theta_trim, phi_trim= phi_trim,
                                         a_phi1=a_phi1, a_phi2=a_phi2, a_beta1=a_beta1, a_beta2 = a_beta2,
                                         a_theta1=a_theta1, a_theta2=a_theta2, a_theta3=a_theta3,a_V1=a_V1,
                                         a_V2=a_V2, a_V3=a_V3)


def dThrust_dVa(Va, Throttle, epsilon=0.50):
    VAM = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    Fx, Mx = VAM.CalculatePropForces(Va, Throttle)
    FxPlus, MxPlus = VAM.CalculatePropForces(Va + epsilon, Throttle)
    dTdVa = (FxPlus - Fx) / epsilon

    return dTdVa


def dThrust_dThrottle(Va, Throttle, epsilon=0.01):
    VAM = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    Fx,Mx = VAM.CalculatePropForces(Va, Throttle)
    FxPlus, MxPlus = VAM.CalculatePropForces(Va, Throttle + epsilon)
    dTdDeltaRT = (FxPlus - Fx) / epsilon

    return dTdDeltaRT
