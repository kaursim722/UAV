import math
import numpy as np
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleAerodynamicsModel():

    def __init__(self, initialSpeed = VPC.InitialSpeed, initialHeight = VPC.InitialDownPosition):

        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.VDM = VehicleDynamicsModel.VehicleDynamicsModel()
        self.VDM.state.pn = VPC.InitialNorthPosition
        self.VDM.state.pe = VPC.InitialEastPosition
        self.VDM.state.pd = VPC.InitialDownPosition
        self.VDM.state.u = VPC.InitialSpeed
        self.wind_Model = WindModel.WindModel()

    def reset(self):
        v_state = States.vehicleState()
        self.VDM = VehicleDynamicsModel.VehicleDynamicsModel()
        self.VDM.state = v_state
        self.VDM.state.pd = self.initialHeight
        self.VDM.state.u = self.initialSpeed
        self.VDM.dot = v_state
        self.wind_Model.reset()
        return

    def getVehicleState(self):
        return self.VDM.state

    def setVehicleState(self, state):
        self.VDM.state = state
        return

#changed from self.VDM.state to self.VDM
    def getVehicleDynamicsModel(self):
        return self.VDM

    def Update(self, controls):
        self.wind_Model.Update()
        V_state = self.VDM.getVehicleState()
        W_state = self.wind_Model.getWind()
        update_force = self.updateForces(V_state, controls, W_state)
        self.VDM.Update(update_force)
        return

    def gravityForces(self, state):
        half = MatrixMath.scalarMultiply(VPC.mass, state.R)
        fg = MatrixMath.multiply(half, [[0],[0],[VPC.g0]])
        fm = Inputs.forcesMoments(Fx = fg[0][0] , Fy= fg[1][0], Fz = fg[2][0])
        return fm

    def sigma(self, a, a0, M):
        first_half = 1.0 + np.exp(-M * (a - a0)) + np.exp(M * (a + a0))
        second_half = (1.0 + np.exp(-M * (a - a0))) * (1.0 + np.exp(M * (a + a0)))
        final_sigma = first_half/second_half
        return final_sigma


    def CalculateCoeff_alpha(self, alpha):
        sig = self.sigma(alpha, VPC.alpha0, VPC.M)
        lift_a = VPC.CL0 + (VPC.CLalpha * alpha)
        drag_a = VPC.CDp + ((VPC.CL0 + VPC.CLalpha * alpha)**2.0)/(math.pi * VPC.e * VPC.AR)
        lift_s = 2.0 * math.sin(alpha) * math.cos(alpha)
        drag_s = 2.0 * (math.sin(alpha) ** 2.0)
        c_l = ((1.0 - sig) * lift_a) + (sig * lift_s)
        c_d = ((1.0 - sig) * drag_a) + (sig * drag_s)
        c_m = VPC.CM0 + (VPC.CMalpha * alpha)
        return c_l, c_d, c_m


    def aeroForces(self, state):
        c = self.CalculateCoeff_alpha(state.alpha)
        if state.Va == 0.0:

            F_lift = 0.0
            F_drag = 0.0
            m = 0.0
            f_y = 0.0
            l = 0.0
            n =  0.0

        else:
            F_lift = (0.5 * VPC.rho * (state.Va**2.0) * VPC.S) * (c[0] +
                    (VPC.CLq * ((VPC.c / (2.0 * state.Va)) * state.q)))
            F_drag = (0.5 * VPC.rho * (state.Va**2.0) * VPC.S) * (c[1] + (VPC.CDq * ((VPC.c / (2.0 * state.Va)) * state.q)))
            m = (0.5 * VPC.rho * (state.Va**2.0) * VPC.S * VPC.c) * (VPC.CM0 +
                    (VPC.CMalpha * state.alpha) + (VPC.CMq * ((VPC.c / (2.0 * state.Va)) * state.q)))
            f_y = (0.5 * VPC.rho * (state.Va**2.0) * VPC.S) * (VPC.CY0 + (VPC.CYbeta * state.beta) +
                    (VPC.CYp * ((VPC.b / (2.0 * state.Va)) * state.p)) + (VPC.CYr * ((VPC.b/(2.0 * state.Va))*state.r)))
            l = (0.5 * VPC.rho * (state.Va**2.0) * VPC.S * VPC.b) * (VPC.Cl0 + (VPC.Clbeta * state.beta) +
                    (VPC.Clp * ((VPC.b / (2.0 * state.Va)) * state.p)) + (VPC.Clr * ((VPC.b/(2.0 * state.Va))*state.r)))
            n = (0.5 * VPC.rho * (state.Va**2.0) * VPC.S * VPC.b) * (VPC.Cn0 + (VPC.Cnbeta * state.beta) +
                    (VPC.Cnp * ((VPC.b / (2.0 * state.Va)) * state.p)) + (VPC.Cnr * ((VPC.b/(2.0 * state.Va))*state.r)))

        m_1 = [[math.cos(state.alpha),-1.0*math.sin(state.alpha)],
               [math.sin(state.alpha),math.cos(state.alpha)]]
        m_2 = [[-1.0*F_drag],
               [-1.0*F_lift]]
        fx_fz = MatrixMath.multiply(m_1, m_2)

        final = Inputs.forcesMoments(Fx=fx_fz[0][0], Fy=f_y, Fz=fx_fz[1][0], Mx=l , My=m , Mz=n)

        return final

    def controlForces(self, state, controls):

        F_lift = 0.5 * VPC.rho * (state.Va**2.0) * VPC.S * VPC.CLdeltaE * controls.Elevator
        F_drag = 0.5 * VPC.rho * (state.Va**2.0) * VPC.S * VPC.CDdeltaE * controls.Elevator
        m = 0.5 * VPC.rho * (state.Va**2.0) * VPC.S * VPC.c * VPC.CMdeltaE * controls.Elevator
        f_y = 0.5 * VPC.rho * (state.Va**2.0) * VPC.S * ((VPC.CYdeltaA * controls.Aileron) + (VPC.CYdeltaR * controls.Rudder))
        l = 0.5 * VPC.rho * (state.Va**2.0) * VPC.S * VPC.b * ((VPC.CldeltaA * controls.Aileron) + (VPC.CldeltaR * controls.Rudder))
        n = 0.5 * VPC.rho * (state.Va**2.0) * VPC.S * VPC.b * ((VPC.CndeltaA * controls.Aileron) + (VPC.CndeltaR * controls.Rudder))

        m_1 = [[math.cos(state.alpha), -math.sin(state.alpha)],
               [math.sin(state.alpha), math.cos(state.alpha)]]
        m_2 = [[-F_drag],
               [-F_lift]]
        fx_fz = MatrixMath.multiply(m_1, m_2)

        calc_tupe = self.CalculatePropForces(state.Va, controls.Throttle)

        final = Inputs.forcesMoments(Fx=fx_fz[0][0] + calc_tupe[0], Fy=f_y, Fz=fx_fz[1][0] , Mx=l+calc_tupe[1], My=m, Mz=n)

        return final

    def CalculatePropForces(self, Va, Throttle):

        Kt = 60.0 / (2.0 * math.pi * VPC.KV)
        Vin = VPC.V_max * Throttle
        a = (VPC.rho * (VPC.D_prop**5.0) * VPC.C_Q0)/(4.0 * (math.pi**2.0))
        b = ((VPC.rho * (VPC.D_prop ** 4.0) * Va * VPC.C_Q1) / (2.0*math.pi)) + ((Kt*Kt) / VPC.R_motor)
        c = (VPC.rho * (VPC.D_prop ** 3.0) * (Va ** 2.0) * VPC.C_Q2) - (Kt * (Vin/VPC.R_motor)) + (Kt * VPC.i0)
        try:
            omega = (-b + math.sqrt((b**2.0) - (4.0 * a * c)))/(2.0 * a)
        except:
            omega = 100.0
        J = (2.0 * math.pi * Va) / (omega * VPC.D_prop)
        CT = VPC.C_T0 + (VPC.C_T1 * J) + (VPC.C_T2 * (J ** 2.0))
        CQ = VPC.C_Q0 + (VPC.C_Q1 * J) + (VPC.C_Q2 * (J ** 2.0))

        f_prop = (VPC.rho * (omega ** 2.0) * (VPC.D_prop**4.0) * CT)/(4.0*(math.pi**2.0))
        m_prop = -1.0*((VPC.rho * (omega ** 2.0) * (VPC.D_prop**5.0) * CQ)/(4.0*(math.pi**2.0)))

        return f_prop, m_prop

    def updateForces(self, state, controls, wind=None):
        if wind == None:
            state.Va = math.hypot(state.u, state.v, state.w)  # Airspeed
            state.alpha = math.atan2(state.w, state.u)  # angle of attack
            if math.isclose(state.Va, 0.0):  # Sideslip Angle, no airspeed
                state.beta = 0.0
            else:
                state.beta = math.asin(state.v / state.Va)  # Sideslip Angle, normal definition
        else:
            air = self.CalculateAirspeed(state, wind)
            state.Va = air[0]
            state.alpha = air[1]
            state.beta = air[2]

        a = self.aeroForces(state)
        g = self.gravityForces(state)
        c = self.controlForces(state, controls)

        new_fx = a.Fx + g.Fx + c.Fx
        new_fy = a.Fy + g.Fy + c.Fy
        new_fz = a.Fz + g.Fz + c.Fz

        new_mx = a.Mx + c.Mx
        new_my = a.My + c.My
        new_mz = a.Mz + c.Mz

        final = Inputs.forcesMoments(Fx=new_fx, Fy=new_fy, Fz=new_fz, Mx=new_mx, My=new_my, Mz=new_mz)

        return final

    def getWindModel(self):
        return self.wind_Model

    def setWindModel(self, windModel):
        self.wind_Model = windModel
        return

    def CalculateAirspeed(self, state, wind):
        xw = math.atan2(wind.We, wind.Wn)
        w = math.hypot(wind.Wn, wind.We, wind.Wd)
        if math.isclose(w, 0.0):
            gamma_w = 0.0
        else:
            gamma_w = -math.asin(wind.Wd / w)

        Rchi_omega = Rotations.euler2DCM(xw, gamma_w, 0.0)
        R_WtoI = MatrixMath.transpose(Rchi_omega)
        wg = MatrixMath.multiply(R_WtoI, [[wind.Wu], [wind.Wv], [wind.Ww]])
        wtot_body = MatrixMath.add([[wind.Wn],[wind.We],[wind.Wd]], wg)
        matrix_1 = MatrixMath.multiply(state.R, wtot_body)

        Vspeed = MatrixMath.subtract([[state.u], [state.v], [state.w]], matrix_1)
        Va = math.hypot(Vspeed[0][0], Vspeed[1][0], Vspeed[2][0])
        alpha = math.atan2(Vspeed[2][0], Vspeed[0][0])

        if math.isclose(Va, 0.0):
            beta = 0.0
        else:
            beta = math.asin(Vspeed[1][0] / Va)

        return Va, alpha, beta


