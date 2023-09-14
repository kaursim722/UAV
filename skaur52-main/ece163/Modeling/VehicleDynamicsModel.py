import math
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

# -------- Citations -------
#   all equations were used from the course files: attitude cheat sheet, code documentation, kinematics cheat sheet
#   kinematic and dynamic equations were taken from Gabriel Elkaim's lecture notes
#


class VehicleDynamicsModel():

    def __init__(self, dT=VPC.dT):
        self.dot = States.vehicleState()
        self.state = States.vehicleState()
        self.dT = dT

    def setVehicleState(self, state):
        self.state = state
        return

    def getVehicleState(self):
        return self.state

    def setVehicleDerivative(self, dot):
        self.dot = dot
        return

    def getVehicleDerivative(self):
        return self.dot

    def reset(self):
        self.setVehicleState(States.vehicleState())
        self.setVehicleDerivative(States.vehicleState())
        return

    def Update(self, forcesMoments):
        state = self.getVehicleState()
        dot = self.derivative(state, forcesMoments)
        self.state = self.IntegrateState(self.dT, state, dot)
        return

    def Rexp(self, dT, state, dot):

        p = state.p + (dot.p * (dT/2.0))
        q = state.q + (dot.q * (dT/2.0))
        r = state.r + (dot.r * (dT/2.0))
        w = math.hypot(p, q, r)
        skew_1 = mm.skew(p, q, r)
        skew_2 = mm.multiply(skew_1, skew_1)

        I_matrix = [[1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]]

        if w >= 0.2:
            divide_1 = (math.sin(w * dT)) / w
            matrix_1 = mm.scalarMultiply(divide_1, skew_1)
            divide_2 = (1.0 - math.cos(w * dT)) / (w * w)
            matrix_2 = mm.scalarMultiply(divide_2, skew_2)

            half_matrix = mm.subtract(I_matrix, matrix_1)
            full_matrix = mm.add(half_matrix, matrix_2)

        else:

            divide_1 = dT - ((dT ** 3.0) * (w ** 2.0) / 6.0) + ((dT ** 5.0) * (w ** 4.0) / 120.0)
            matrix_1 = mm.scalarMultiply(divide_1, skew_1)
            divide_2 = (dT ** 2.0) / 2.0 - ((dT ** 4.0) * (w ** 2.0) / 24.0) + ((dT ** 6.0) * (w ** 4.0) / 720.0)
            matrix_2 = mm.scalarMultiply(divide_2, skew_2)

            half_matrix = mm.subtract(I_matrix, matrix_1)
            full_matrix = mm.add(half_matrix, matrix_2)

        return full_matrix

    def derivative(self, state, forcesMoments):
        dot = States.vehicleState()

        p = state.p
        q = state.q
        r = state.r

        yaw = state.yaw
        pitch = state.pitch
        roll = state.roll

        u = state.u
        v = state.v
        w = state.w

        skew_m = mm.skew(p, q, r)
        skew_negate = mm.scalarMultiply(-1.0, skew_m)
        skew_final = mm.multiply(skew_negate, [[u], [v], [w]])
        state.R = Rotations.euler2DCM(yaw, pitch, roll)
        forces_mass = mm.scalarMultiply(1.0 / VPC.mass, [[forcesMoments.Fx], [forcesMoments.Fy], [forcesMoments.Fz]])
        final_uvw = mm.add(forces_mass, skew_final)

        ypr_matrix = [[1.0, math.sin(roll) * math.tan(pitch), math.cos(roll) * math.tan(pitch)],
                      [0.0, math.cos(roll), -math.sin(roll)],
                      [0.0, (math.sin(roll)) / (math.cos(pitch)), (math.cos(roll)) / (math.cos(pitch))]]
        deriv_ypr = mm.multiply(ypr_matrix, [[p], [q], [r]])

        j_omega = mm.multiply(VPC.Jbody, [[p], [q], [r]])
        skew_jw = mm.multiply(skew_negate, j_omega)
        dpqr_matrix = mm.add(skew_jw, [[forcesMoments.Mx], [forcesMoments.My], [forcesMoments.Mz]])
        final_dpqr = mm.multiply(VPC.JinvBody, dpqr_matrix)

        R_deriv = mm.multiply(skew_negate, state.R)

        ned_deriv = mm.multiply(mm.transpose(state.R), [[u], [v], [w]])

        chi = math.atan2(ned_deriv[1][0], ned_deriv[2][0])

        dot.pn = ned_deriv[0][0]
        dot.pe = ned_deriv[1][0]
        dot.pd = ned_deriv[2][0]
        dot.u = final_uvw[0][0]
        dot.v = final_uvw[1][0]
        dot.w = final_uvw[2][0]
        dot.yaw = deriv_ypr[2][0]
        dot.pitch = deriv_ypr[1][0]
        dot.roll = deriv_ypr[0][0]
        dot.p = final_dpqr[0][0]
        dot.q = final_dpqr[1][0]
        dot.r = final_dpqr[2][0]
        dot.dcm = R_deriv
        dot.chi = chi

        return dot

    def ForwardEuler(self, dT, state, dot):
        n = state.pn + dot.pn * dT
        e = state.pe + dot.pe * dT
        d = state.pd + dot.pd * dT

        u = state.u + dot.u * dT
        v = state.v + dot.v * dT
        w = state.w + dot.w * dT

        p = state.p + dot.p * dT
        q = state.q + dot.q * dT
        r = state.r + dot.r * dT

        return States.vehicleState(pn=n, pe=e, pd=d, u=u, v=v, w=w, p=p, q=q, r=r)

    def IntegrateState(self, dT, state, dot):

        p = self.ForwardEuler(dT, state, dot).p
        q = self.ForwardEuler(dT, state, dot).q
        r = self.ForwardEuler(dT, state, dot).r

        n = self.ForwardEuler(dT, state, dot).pn
        e = self.ForwardEuler(dT, state, dot).pe
        d = self.ForwardEuler(dT, state, dot).pd

        u = self.ForwardEuler(dT, state, dot).u
        v = self.ForwardEuler(dT, state, dot).v
        w = self.ForwardEuler(dT, state, dot).w

        R_k1 = mm.multiply(self.Rexp(dT, state, dot), state.R)

        sim_vState = States.vehicleState(pn=n, pe=e, pd=d,
                                   u=u, v=v, w=w,
                                   p=p, q=q, r=r, dcm=R_k1)

        sim_vState.alpha = state.alpha
        sim_vState.beta = state.beta
        sim_vState.Va = state.Va
        sim_vState.chi = math.atan2(dot.pe, dot.pn)

        return sim_vState
