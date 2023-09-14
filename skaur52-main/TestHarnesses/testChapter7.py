"""This file is a test harness for the module VehiclePerturbationModels.

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter5.py (from the root directory) -or-
python testChapter5.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehiclePerturbationModels module"""

# %% Initialization of test harness and helpers:

import math

import sys

sys.path.append("..")  # python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Modeling.WindModel as WM
import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Sensors.SensorsModel as Sensors

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda a, b: math.isclose(a, b, abs_tol=1e-12)


def compareVectors(a, b):
    """A quick tool to compare two vectors"""
    el_close = [isclose(a[i][0], b[i][0]) for i in range(2)]
    return all(el_close)



# of course, you should test your testing tools too:
assert (compareVectors([[0], [0], [-1]], [[1e-13], [0], [-1 + 1e-9]]))
assert (not compareVectors([[0], [0], [-1]], [[1e-11], [0], [-1]]))
assert (not compareVectors([[1e8], [0], [-1]], [[1e8 + 1], [0], [-1]]))

failed = []
passed = []


def evaluateTest(test_name, boolean):
    """evaluateTest prints the output of a test and adds it to one of two
    global lists, passed and failed, which can be printed later"""
    if boolean:
        print(f"   passed {test_name}")
        passed.append(test_name)
    else:
        print(f"   failed {test_name}")
        failed.append(test_name)
    return boolean


# %% PUT A TEST HERE?
# vTrim = VehicleTrim.VehicleTrim()
# Vastar = 25.0
# Gammastar = math.radians(6.0)
# Kappastar = -1.0 / 150.0
#
# check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
# if check:
#     print("Optimization successful")
# else:
#     print("Model converged outside of valid inputs, change parameters and try again")
#
# tF = VPM.CreateTransferFunction(
#     vTrim.getTrimState(),
#     vTrim.getTrimControls())


testingVDM = VDM.VehicleDynamicsModel()
testingVDM.state.p = 0.5
testingVDM.state.q = 15.35
testingVDM.state.r = 26.45
testingVDM.state.pn = 0.3
testingVDM.state.pe = 0.2
testingVDM.state.pd = -150.0

stateT = testingVDM.getVehicleState()
dotT = testingVDM.getVehicleDerivative()
sensorT = Sensors.SensorsModel()


print("Sensors Model test")
cur_test = "testing mags"
mag_x, mag_y, mag_z = sensorT.updateMagsTrue(stateT)
observe = [[mag_x],[mag_y],[mag_z]]
expect = [[22750.0], [5286.8], [41426.3]]
if compareVectors(observe,expect):
 print("passed!")
else:
 print("failed :(")

cur_test = "testing gyro"
gyro_x, gyro_y, gyro_z = sensorT.updateGyrosTrue(stateT)
observe = [[gyro_x],[gyro_y],[gyro_z]]
expect = [[0.5], [15.35], [26.45]]
if compareVectors(observe,expect):
 print("passed!")
else:
 print("failed :(")

cur_test = "testing accel"
accel_x, accel_y, accel_z = sensorT.updateAccelsTrue(stateT, dotT)
observe = [[accel_x],[accel_y],[accel_z]]
expect = [[0.0], [0.0], [-9.81]]
if compareVectors(observe,expect):
 print("passed!")
else:
 print("failed :(")

cur_test = "testing pressure"
baro, pitot = sensorT.updatePressureSensorsTrue(stateT)
observe = [[baro], [pitot]]
expect = [[99458.8437], [0.0]]
if compareVectors(observe,expect):
 print("passed!")
else:
 print("failed :(")

cur_test = "testing update"
sensorT.update()
sens = sensorT.SensorsTrue

observe = [[sens.gyro_x], [sens.gyro_y], [sens.gyro_z], [sens.accel_x], [sens.accel_y], [sens.accel_z],
           [sens.mag_x], [sens.mag_y], [sens.mag_z]]

expect = [[0.0], [0.0], [0.0], [0.0], [0.0], [-9.81], [22750.0], [5286.8], [41426.3]]
if compareVectors(observe,expect):
 print("passed!")
else:
 print("failed :(")

cur_test = "testing gps"
n, e, d, speed, course = sensorT.updateGPSTrue(stateT, dotT)
observe = [[n], [e], [d], [speed], [course]]
expect = [[0.3], [0.2], [150.0], [0.0], [0.0]]
if compareVectors(observe,expect):
 print("passed!")
else:
 print("failed :(")

cur_test = "testing reset"
sensorT.reset()
reset = sensorT
observe = [[sens.gyro_x], [sens.gyro_y], [sens.gyro_z], [sens.accel_x], [sens.accel_y], [sens.accel_z],
           [sens.mag_x], [sens.mag_y], [sens.mag_z]]
expect = [[0.0], [0.0], [0.0], [0.0], [0.0], [-9.81], [22750.0], [5286.8], [41426.3]]
if compareVectors(observe,expect):
 print("passed!")
else:
 print("failed :(")

# %% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
    print(f"Failed {len(failed)}/{total} tests:")
    [print("   " + test) for test in failed]