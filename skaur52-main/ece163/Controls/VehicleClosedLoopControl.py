import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule

class VehicleClosedLoopControl():

    def __init__(self, dT=0.01, rudderControlSource='SIDESLIP'):
        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        self.trimIn = Inputs.controlInputs()
        self.control_G = Controls.controlGains()
        self.aeroIn = Inputs.controlInputs()
        self.rollFromCourse = PIControl()
        self.rudderFromSideslip = PIControl()
        self.throttleFromAirspeed = PIControl()
        self.pitchFromAltitude = PIControl()
        self.pitchFromAirspeed = PIControl()
        self.elevatorFromPitch = PDControl()
        self.aileronFromRoll = PIDControl()
        self.mode = Controls.AltitudeStates.HOLDING
        self.dT = dT
        return

    def Update(self, referenceCommands=Controls.referenceCommands):
        vehicle_state = self.getVehicleState()
        u = self.UpdateControlCommands(referenceCommands, vehicle_state)
        self.VAM.Update(u)
        return

    def UpdateControlCommands(self, referenceCommands, state):
        control_in = Inputs.controlInputs()
        currentState = self.mode
        altitude = -state.pd
        holding = Controls.AltitudeStates.HOLDING
        climbing = Controls.AltitudeStates.CLIMBING
        descending = Controls.AltitudeStates.DESCENDING
        lower_thresh = referenceCommands.commandedAltitude - VPC.altitudeHoldZone
        upper_thresh = referenceCommands.commandedAltitude + VPC.altitudeHoldZone

        commandedCourse = referenceCommands.commandedCourse
        commandedAirspeed = referenceCommands.commandedAirspeed
        commandedAltitude = referenceCommands.commandedAltitude

        course_error = commandedCourse - state.chi
        if course_error >= math.pi:
            state.chi = state.chi + (2.0 * math.pi)
        elif course_error <= -math.pi:
            state.chi = state.chi - (2.0 * math.pi)

        roll_course = self.rollFromCourse.Update(commandedCourse, state.chi)
        referenceCommands.commandedRoll = roll_course
        aileron_roll = self.aileronFromRoll.Update(roll_course, state.roll, state.p)
        sideslip_rudder = self.rudderFromSideslip.Update(0.0, state.beta)
        air_throttle = self.throttleFromAirspeed.Update(commandedAirspeed, state.Va)
        air_pitch = self.pitchFromAirspeed.Update(commandedAirspeed, state.Va)
        pitch_alt = self.pitchFromAltitude.Update(commandedAltitude, -state.pd)
        throttleCommand = 0.0
        pitchCommand = 0.0


        if(currentState == holding):
            throttleCommand = air_throttle
            pitchCommand = pitch_alt
            if(altitude > upper_thresh):
                currentState = descending
                throttleCommand = VPC.minControls.Throttle
                pitchCommand = air_pitch
                self.pitchFromAirspeed.resetIntegrator()
            elif(altitude < lower_thresh):
                currentState = climbing
                throttleCommand = VPC.maxControls.Throttle
                pitchCommand = air_pitch
                self.pitchFromAirspeed.resetIntegrator()


        elif(currentState == descending):
            throttleCommand = VPC.minControls.Throttle
            pitchCommand = air_pitch
            if (lower_thresh < altitude < upper_thresh):
                currentState = holding
                throttleCommand = air_throttle
                pitchCommand = pitch_alt
                self.pitchFromAltitude.resetIntegrator()


        elif(currentState == climbing):
            throttleCommand = VPC.maxControls.Throttle
            pitchCommand = air_pitch
            if (lower_thresh < altitude < upper_thresh):
                currentState = holding
                throttleCommand = air_throttle
                pitchCommand = pitch_alt
                self.pitchFromAltitude.resetIntegrator()

        elevator_pitch = self.elevatorFromPitch.Update(pitchCommand, state.pitch, state.q)
        referenceCommands.commandedPitch = pitchCommand

        control_in.Throttle = throttleCommand
        control_in.Elevator = elevator_pitch
        control_in.Aileron = aileron_roll
        control_in.Rudder = sideslip_rudder

        return control_in

    def getControlGains(self):
        return self.control_G

    def getTrimInputs(self):
        return self.trimIn

    def getVehicleAerodynamicsModel(self):
        return self.VAM

    def getVehicleControlSurfaces(self):
        return self.aeroIn

    def getVehicleState(self):
        return self.VAM.getVehicleState()

    def reset(self):
        self.rollFromCourse.resetIntegrator()
        self.aileronFromRoll.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.VAM.reset()
        return

    def setControlGains(self, controlGains=Controls.controlGains()):
        self.control_G = controlGains
        self.rollFromCourse.setPIGains(dT=self.dT, kp=self.control_G.kp_course, ki=self.control_G.ki_course, trim=0.0,
                                       lowLimit=-math.radians(VPC.bankAngleLimit), highLimit=math.radians(VPC.bankAngleLimit))
        self.rudderFromSideslip.setPIGains(dT=self.dT, kp=self.control_G.kp_sideslip, ki=self.control_G.ki_sideslip,
                                           trim=self.trimIn.Rudder, lowLimit=VPC.minControls.Rudder, highLimit=VPC.maxControls.Rudder)
        self.throttleFromAirspeed.setPIGains(dT=self.dT, kp=self.control_G.kp_SpeedfromThrottle, ki=self.control_G.ki_SpeedfromThrottle,
                                           trim=self.trimIn.Throttle, lowLimit=VPC.minControls.Throttle,
                                           highLimit=VPC.maxControls.Throttle)
        self.elevatorFromPitch.setPDGains(kp=self.control_G.kp_pitch, kd=self.control_G.kd_pitch,
                                           trim=self.trimIn.Elevator, lowLimit=VPC.minControls.Elevator,
                                           highLimit=VPC.maxControls.Elevator)
        self.pitchFromAltitude.setPIGains(dT=self.dT, kp=self.control_G.kp_altitude, ki=self.control_G.ki_altitude,
                                           trim=0.0, lowLimit=-math.radians(VPC.pitchAngleLimit),
                                           highLimit=math.radians(VPC.pitchAngleLimit))
        self.pitchFromAirspeed.setPIGains(dT=self.dT, kp=self.control_G.kp_SpeedfromElevator, ki=self.control_G.ki_SpeedfromElevator,
                                           trim=0.0, lowLimit=-math.radians(VPC.pitchAngleLimit),
                                           highLimit=math.radians(VPC.pitchAngleLimit))
        self.aileronFromRoll.setPIDGains(dT=self.dT, kp=self.control_G.kp_roll, kd=self.control_G.kd_roll, ki=self.control_G.ki_roll,
                                           trim=self.trimIn.Aileron, lowLimit=VPC.minControls.Aileron,
                                           highLimit=VPC.maxControls.Aileron)

        return

    def setTrimInputs(self, trimInputs=Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):
        self.trimIn = trimInputs
        return

    def setVehicleState(self, state):
        self.VAM.setVehicleState(state)
        return



class PDControl():

    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

        return

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        err = command - current
        u_unsat = (self.kp * err) - (self.kd * derivative) + self.trim
        u = u_unsat
        if u < self.lowLimit:
            u = self.lowLimit
        elif u > self.highLimit:
            u = self.highLimit
        else:
            u = u_unsat
        return u


    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return



class PIControl():

    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.previousError = 0.0
        return

    def Update(self, command=0.0, current=0.0):
        err = command - current
        self.accumulator = self.accumulator + (self.dT / 2.0) * (err + self.previousError)
        u_unsat = (self.kp * err) + (self.ki * self.accumulator) + self.trim

        if u_unsat < self.lowLimit:
            u = self.lowLimit
            self.accumulator = self.accumulator - (self.dT / 2.0) * (err + self.previousError)
        elif u_unsat > self.highLimit:
            u = self.highLimit
            self.accumulator = self.accumulator - (self.dT / 2.0) * (err + self.previousError)
        else:
            u = u_unsat
        self.previousError = err
        return u


    def resetIntegrator(self):
        self.accumulator = 0.0
        self.previousError = 0.0
        return

    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return



class PIDControl():

    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.previousError = 0.0
        return

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        err = command - current
        self.accumulator = self.accumulator + (self.dT / 2.0) * (err + self.previousError)
        u_unsat = (self.kp * err) + (self.ki * self.accumulator) - (self.kd * derivative) + self.trim

        if u_unsat < self.lowLimit:
            u = self.lowLimit
            self.accumulator = self.accumulator - (self.dT / 2.0) * (err + self.previousError)
        elif u_unsat > self.highLimit:
            u = self.highLimit
            self.accumulator = self.accumulator - (self.dT / 2.0) * (err + self.previousError)
        else:
            u = u_unsat
        self.previousError = err
        return u


    def resetIntegrator(self):
        self.accumulator = 0.0
        self.previousError = 0.0
        return

    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

