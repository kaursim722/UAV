import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel


class GaussMarkov():

    def __init__(self, dT=0.01, tau=1000000.0, eta=0.0):
        self.dT = dT
        self.tau = tau
        self.eta = eta
        self.v = 0.0

        return

    def reset(self):
        self.v = 0.0

        return

    def update(self, vnoise=None):
        if vnoise is None:
            vnoise = random.gauss(0.0, self.eta)
        equate = ((math.exp(-self.dT/self.tau)) * self.v) + vnoise
        self.v = equate

        return self.v


class GaussMarkovXYZ():

    def __init__(self, dT=0.01, tauX=1000000.0, etaX=0.0, tauY=None, etaY=None, tauZ=None, etaZ=None):
        self.dT = dT
        if tauY is None and etaY is None:
            tauY = tauX
            etaY = etaX

            if tauZ is None and etaZ is None:
                tauZ = tauX
                etaZ = etaX

        else:
            if tauZ is None and etaZ is None:
                tauZ = tauY
                etaZ = etaY

        self.gauss_x = GaussMarkov(dT, tauX, etaX)
        self.gauss_y = GaussMarkov(dT, tauY, etaY)
        self.gauss_z = GaussMarkov(dT, tauZ, etaZ)

        return

    def reset(self):
        self.gauss_x.reset()
        self.gauss_y.reset()
        self.gauss_z.reset()

        return

    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):
        x = self.gauss_x.update(vXnoise)
        y = self.gauss_y.update(vYnoise)
        z = self.gauss_z.update(vZnoise)

        return x, y, z


class SensorsModel():
    def __init__(self, aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(), taugyro=400.0, etagyro=0.0012740903539558606,
                   tauGPS = 1100.0, etaGPSHorizontal=0.21, etaGPSVertical=0.4, gpsUpdateHz=1.0):
        self.aeroModel = aeroModel
        self.DM = aeroModel.getVehicleDynamicsModel()
        self.dT = self.DM.dT
        self.updateT = 0.0
        self.SensorsNoisy = Sensors.vehicleSensors()
        self.SensorsTrue = Sensors.vehicleSensors()
        self.SensorsBiases = self.initializeBiases()
        self.SensorsSigmas = self.initializeSigmas()
        self.GPS = GaussMarkovXYZ(1/gpsUpdateHz, tauGPS, etaGPSHorizontal, tauGPS, etaGPSHorizontal, tauGPS, etaGPSVertical)
        self.gyro = GaussMarkovXYZ(dT= self.dT, tauX=taugyro, etaX= etagyro)

        return

    def getSensorsNoisy(self):
        return self.SensorsNoisy

    def getSensorsTrue(self):
        return self.SensorsTrue

    def initializeBiases(self, gyroBias=0.08726646259971647, accelBias=0.9810000000000001, magBias=500.0, baroBias=100.0, pitotBias=20.0):
        SB = Sensors.vehicleSensors()

        SB.gyro_x = gyroBias * random.uniform(-1.0, 1.0)
        SB.gyro_y = gyroBias * random.uniform(-1.0, 1.0)
        SB.gyro_z = gyroBias * random.uniform(-1.0, 1.0)

        SB.accel_x = accelBias * random.uniform(-1.0, 1.0)
        SB.accel_y = accelBias * random.uniform(-1.0, 1.0)
        SB.accel_z = accelBias * random.uniform(-1.0, 1.0)

        SB.mag_x = magBias * random.uniform(-1.0, 1.0)
        SB.mag_y = magBias * random.uniform(-1.0, 1.0)
        SB.mag_z = magBias * random.uniform(-1.0, 1.0)

        SB.baro = baroBias * random.uniform(-1.0, 1.0)
        SB.pitot = pitotBias * random.uniform(-1.0, 1.0)

        return SB

    def initializeSigmas(self, gyroSigma=0.002617993877991494, accelSigma=0.24525000000000002, magSigma=25.0, baroSigma=10.0,
                         pitotSigma=2.0, gpsSigmaHorizontal=0.4, gpsSigmaVertical=0.7, gpsSigmaSOG=0.05, gpsSigmaCOG=0.002):
       SS = Sensors.vehicleSensors()

       SS.gyro_x = gyroSigma
       SS.gyro_y = gyroSigma
       SS.gyro_z = gyroSigma

       SS.accel_x = accelSigma
       SS.accel_y = accelSigma
       SS.accel_z = accelSigma

       SS.mag_x = magSigma
       SS.mag_y = magSigma
       SS.mag_z = magSigma

       SS.baro = baroSigma
       SS.pitot = pitotSigma

       SS.gps_n = gpsSigmaHorizontal
       SS.gps_e = gpsSigmaHorizontal

       SS.gps_sog = gpsSigmaSOG
       SS.gps_cog = gpsSigmaCOG
       SS.gps_alt = gpsSigmaVertical

       return SS

    def reset(self):
        self.SensorsNoisy = Sensors.vehicleSensors()
        self.SensorsTrue = Sensors.vehicleSensors()
        self.SensorsSigmas = self.initializeSigmas()
        self.SensorsBiases = self.initializeBiases()
        self.GPS.reset()
        self.gyro.reset()
        return

    def update(self):
        state = self.DM.state
        dot = self.DM.dot
        self.SensorsTrue = self.updateSensorsTrue(self.SensorsTrue, state, dot)
        self.SensorsNoisy = self.updateSensorsNoisy(self.SensorsTrue, self.SensorsNoisy, self.SensorsBiases, self.SensorsSigmas)
        self.updateT = self.updateT + 1.0
        return

    def updateAccelsTrue(self, state, dot):
        g = [[0],[0],[VPC.g0]]
        dot_uvw = [[dot.u],[dot.v],[dot.w]]
        state_uvw = [[state.u],[state.v],[state.w]]
        skew_matrix = MatrixMath.skew(state.p, state.q, state.r)
        g_matrix = MatrixMath.multiply(state.R, g)
        skew = MatrixMath.multiply(skew_matrix, state_uvw)
        adding = MatrixMath.add(dot_uvw, skew)
        matrix = MatrixMath.subtract(adding, g_matrix)

        return matrix[0][0], matrix[1][0], matrix[2][0]

    def updateGPSTrue(self, state, dot):
        n = state.pn
        e = state.pe
        d = -state.pd
        speed = math.hypot(dot.pe, dot.pn)
        course = math.atan2(dot.pe, dot.pn)
        return n, e, d, speed, course

    def updateGyrosTrue(self, state):
        gyrox = state.p
        gyroy = state.q
        gyroz = state.r
        return gyrox, gyroy, gyroz

    def updateMagsTrue(self, state):
        mag_field = VSC.magfield
        matrix = MatrixMath.multiply(state.R, mag_field)
        return matrix[0][0], matrix[1][0], matrix[2][0]

    def updatePressureSensorsTrue(self, state):
        pitot = VPC.rho / 2.0 * state.Va ** 2.0
        baro = VSC.Pground + (VPC.rho * VPC.g0 * state.pd)
        return baro, pitot

    def updateSensorsNoisy(self, trueSensors=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                                                            accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,
                                                            gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0),
                                                            noisySensors=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                                                            accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,
                                                            gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0),
                                                            sensorBiases=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                                                            accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,
                                                            gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0),
                                                            sensorSigmas=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                                                            accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,
                                                            gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0)):

       NS = Sensors.vehicleSensors()
       NS.gyro_x = trueSensors.gyro_x + noisySensors.gyro_x + sensorBiases.gyro_x + sensorSigmas.gyro_x
       NS.accel_x = trueSensors.accel_x + noisySensors.accel_x + sensorBiases.accel_x + sensorSigmas.accel_x

       return NS


    def updateSensorsTrue(self, prevTrueSensors, state, dot):

        output = Sensors.vehicleSensors()

        gyro_x, gyro_y, gyro_z = self.updateGyrosTrue(state)
        output.gyro_x = gyro_x
        output.gyro_y = gyro_y
        output.gyro_z = gyro_z

        accel_x, accel_y, accel_z = self.updateAccelsTrue(state, dot)
        output.accel_x = accel_x
        output.accel_y = accel_y
        output.accel_z = accel_z

        baro, pitot = self.updatePressureSensorsTrue(state)

        if self.updateT%self.dT:

            n, e, d, speed, course = self.updateGPSTrue(state, dot)

            output.gps_n = n
            output.gps_e = e
            output.gps_alt = d
            output.gps_cog = course
            output.gps_sog = speed

        else:

            output.gps_n = self.SensorsTrue.gps_n
            output.gps_e = self.SensorsTrue.gps_e
            output.gps_alt = self.SensorsTrue.gps_alt
            output.gps_cog = self.SensorsTrue.gps_cog
            output.gps_sog = self.SensorsTrue.gps_sog

        output.baro = baro
        output.pitot = pitot
        n, e, d, speed, course = self.updateGPSTrue(state, dot)
        output.gps_n = n
        output.gps_e = e
        output.gps_alt = d
        output.gps_cog = course
        output.gps_sog = speed

        mag_x, mag_y, mag_z = self.updateMagsTrue(state)
        output.mag_x = mag_x
        output.mag_y = mag_y
        output.mag_z = mag_z

        self.updateT = self.updateT + 1.0

        return output

