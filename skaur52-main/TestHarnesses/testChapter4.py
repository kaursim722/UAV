"""This file is a test harness for the module VehicleAerodynamicsModel.

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter4.py (from the root directory) -or-
python testChapter4.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleAerodynamicsModel module"""

#%% Initialization of test harness and helpers:

import math

import sys

sys.path.append("../..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Modeling.VehicleAerodynamicsModel as VAM
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(len(b))]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))



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


#%% gravityForces():
print("Beginning testing of VDM.gravityForces")

testVAM = VAM.VehicleAerodynamicsModel()
testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testDot = testVAM.gravityForces(testState)

resultPdot = [[testDot.Fx], [testDot.Fy], [testDot.Fz], [testDot.Mx], [testDot.My], [testDot.Mz]]
expectedPdot = [[0.0], [0.0], [107.91000000000001], [0.0], [0.0], [0.0]]
print("Test for gravityForces:")

if compareVectors(resultPdot,expectedPdot):
 print("passed!")
else:
 print("failed :(")


#%% CalculateCoeff_alpha():
print("Beginning testing of VDM.CalculateCoeff_alpha()")

testVAM = VAM.VehicleAerodynamicsModel()
testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testDot = testVAM.CalculateCoeff_alpha(testState.alpha)


resultPdot = [[testDot[0]], [testDot[1]], [testDot[2]]]
expectedPdot = [[0.2299999999730887], [0.06122729464970145], [0.0135]]
print("Test for CalculateCoeff_alpha:")

if compareVectors(resultPdot,expectedPdot):
 print("passed!")
else:
 print("failed :(")


#%% aeroForces():
print("Beginning testing of VDM.aeroForces()")

testVAM = VAM.VehicleAerodynamicsModel()
testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testDot = testVAM.aeroForces(testState)

resultPdot = [[testDot.Fx], [testDot.Fy], [testDot.Fz], [testDot.Mx], [testDot.My], [testDot.Mz]]
expectedPdot = [[0.0], [0], [0.0], [0], [0], [0]]
print("Test for aeroForces:")

if compareVectors(resultPdot,expectedPdot):
 print("passed!")
else:
 print("failed :(")


#%% controlForces():
print("Beginning testing of VDM.controlForces()")

testVAM = VAM.VehicleAerodynamicsModel()
testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testControl = Inputs.controlInputs()
testDot = testVAM.controlForces(testState, testControl)

resultPdot = [[testDot.Fx], [testDot.Fy], [testDot.Fz], [testDot.Mx], [testDot.My], [testDot.Mz]]
expectedPdot = [[21.817680754436033], [0.0], [0.0], [-0.6194943564776727], [-0.0], [-0.0]]
print("Test for controlForces:")

if compareVectors(resultPdot,expectedPdot):
 print("passed!")
else:
 print("failed :(")


#%% CalculatePropForces():
print("Beginning testing of VDM.CalculatePropForces()")

testVAM = VAM.VehicleAerodynamicsModel()
testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testControl = Inputs.controlInputs()
testDot1, testDot2 = testVAM.CalculatePropForces(testState.Va, testControl.Throttle)


resultPdot = [[testDot1], [testDot2]]
expectedPdot = [[21.817680754436033], [-0.6194943564776727]]
print("Test for CalculatePropForces:")

if compareVectors(resultPdot, expectedPdot):
 print("passed!")
else:
 print("failed :(")


#%% updateForces():
print("Beginning testing of VDM.updateForces()")

testVAM = VAM.VehicleAerodynamicsModel()
testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testControl = Inputs.controlInputs()
testDot = testVAM.updateForces(testState, testControl, testFm)


resultPdot = [[testDot.Fx], [testDot.Fy], [testDot.Fz], [testDot.Mx], [testDot.My], [testDot.Mz]]
expectedPdot = [[21.817680754436033], [0.0], [107.91000000000001], [-0.6194943564776727], [0.0], [0.0]]
print("Test for updateForces:")

if compareVectors(resultPdot,expectedPdot):
 print("passed!")
else:
 print("failed :(")


#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]