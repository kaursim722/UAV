import math
import random
from ..Containers import States
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC
from ..Containers.Inputs import drydenParameters



class WindModel:
    def __init__(self, dT=VPC.dT, Va=VPC.InitialSpeed, drydenParamters=VPC.DrydenNoWind):
        self.dT = dT
        self.Va = Va
        self.phi_u = [[0.0]]
        self.gamma_u = [[0.0]]
        self.H_u = [[0.0]]

        self.phi_v = [[0.0, 0.0], [0.0, 0.0]]
        self.gamma_v = [[0.0], [0.0]]
        self.H_v = [[0.0, 0.0]]

        self.phi_w = [[0.0, 0.0], [0.0, 0.0]]
        self.gamma_w = [[0.0], [0.0]]
        self.H_w = [[0.0, 0.0]]

        self.xu = [[0.0]]
        self.xv = [[0.0], [0.0]]
        self.xw = [[0.0], [0.0]]

        self.drydenParamters = drydenParamters
        self.wind_State = States.windState()
        self.CreateDrydenTransferFns(dT, Va, self.drydenParamters)

    def setWind(self, windState):
        self.wind_State = windState
        return

    def getWind(self):
        return self.wind_State

    def reset(self):
        self.xu = [[0.0]]
        self.xv = [[0.0], [0.0]]
        self.xw = [[0.0], [0.0]]
        self.wind_State = States.windState()
        return

    def CreateDrydenTransferFns(self, dT, Va, drydenParameters):
        Lu = drydenParameters.Lu
        Lv = drydenParameters.Lv
        Lw = drydenParameters.Lw

        if drydenParameters == VPC.DrydenNoWind:
            phi_u = [[1.0]]
            phi_v = [[1.0, 0.0], [0.0, 1.0]]
            phi_w = [[1.0, 0.0], [0.0, 1.0]]
            gamma_u = [[0.0]]
            gamma_v = [[0.0], [0.0]]
            gamma_w = [[0.0], [0.0]]
            H_u = [[1.0]]
            H_v = [[1.0, 1.0]]
            H_w = [[1.0, 1.0]]
        else:
            if Va <= 0:
                raise ArithmeticError
            else:

                # equations for u
                phi_u = [[math.e ** ((-Va / Lu) * dT)]]
                gamma_u = [[(Lu / Va) * (1.0 - (math.e ** (-(Va / Lu) * dT)))]]
                H_u = [[drydenParameters.sigmau * math.sqrt((2 * Va) / (math.pi * Lu))]]

                # equations for v
                v_half = (math.e ** ((-Va / Lv) * dT))

                phiv_matrix = [[(1.0 - ((Va / Lv) * dT)), (-(Va / Lv) ** 2.0 * dT)], [dT, 1.0 + ((Va / Lv) * dT)]]
                phi_v = MatrixMath.scalarMultiply(v_half, phiv_matrix)

                gammav_matrix = [[dT], [((Lv / Va) ** 2.0) * ((math.e ** ((Va / Lv) * dT)) - 1.0) - (Lv / Va) * dT]]
                gamma_v = MatrixMath.scalarMultiply(v_half, gammav_matrix)

                Hv_half = drydenParameters.sigmav * math.sqrt((3.0 * Va) / (math.pi * Lv))
                Hv_matrix = [[1.0, (Va) / (math.sqrt(3.0) * Lv)]]
                H_v = MatrixMath.scalarMultiply(Hv_half, Hv_matrix)

                # equations for w
                w_half = (math.e ** ((-Va / Lw) * dT))

                phiw_matrix = [[(1.0 - ((Va / Lw) * dT)), (-(Va / Lw) ** 2.0) * dT], [dT, 1.0 + ((Va / Lw) * dT)]]
                phi_w = MatrixMath.scalarMultiply(w_half, phiw_matrix)

                gammaw_matrix = [[dT], [((Lw / Va) ** 2.0) * ((math.e ** ((Va / Lw) * dT)) - 1.0) - (Lw / Va) * dT]]
                gamma_w = MatrixMath.scalarMultiply(w_half, gammaw_matrix)

                Hw_half = drydenParameters.sigmaw * math.sqrt((3.0 * Va) / (math.pi * Lw))
                Hw_matrix = [[1.0, (Va) / (math.sqrt(3.0) * Lw)]]
                H_w = MatrixMath.scalarMultiply(Hw_half, Hw_matrix)

        self.phi_u = phi_u
        self.phi_v = phi_v
        self.phi_w = phi_w
        self.gamma_u = gamma_u
        self.gamma_v = gamma_v
        self.gamma_w = gamma_w
        self.H_u = H_u
        self.H_v = H_v
        self.H_w = H_w

        return

    def setWindModelParameters(self, Wn=0.0, We=0.0, Wd=0.0, drydenParamters=VPC.DrydenNoWind):
        self.wind_State.Wn = Wn
        self.wind_State.We = We
        self.wind_State.Wd = Wd
        self.CreateDrydenTransferFns(self.dT, self.Va, drydenParamters)
        return

    def getDrydenTransferFns(self):
        return self.phi_u, self.gamma_u, self.H_u, self.phi_v, self.gamma_v, self.H_v, self.phi_w, self.gamma_w, self.H_w

    def Update(self, uu=None, uv=None, uw=None):
        if uu is None:
            uu = random.gauss(0, 1)
        if uv is None:
            uv = random.gauss(0, 1)
        if uw is None:
            uw = random.gauss(0, 1)

        xneg_u = [[0.0]]
        xneg_v = [[0.0], [0.0]]
        xneg_w = [[0.0], [0.0]]

        first_u = MatrixMath.multiply(self.phi_u, xneg_u)
        second_u = MatrixMath.scalarMultiply(uu, self.gamma_u)
        xpos_u = MatrixMath.add(first_u, second_u)

        first_v = MatrixMath.multiply(self.phi_v, xneg_v)
        second_v = MatrixMath.scalarMultiply(uv, self.gamma_v)
        xpos_v = MatrixMath.add(first_v, second_v)

        first_w = MatrixMath.multiply(self.phi_w, xneg_w)
        second_w = MatrixMath.scalarMultiply(uw, self.gamma_w)
        xpos_w = MatrixMath.add(first_w, second_w)

        self.xu = xpos_u
        self.xv = xpos_v
        self.xw = xpos_w

        self.wind_State.Wu = MatrixMath.multiply(self.H_u, xpos_u)[0][0]
        self.wind_State.Wv = MatrixMath.multiply(self.H_v, xpos_v)[0][0]
        self.wind_State.Ww = MatrixMath.multiply(self.H_w, xpos_w)[0][0]

        return
