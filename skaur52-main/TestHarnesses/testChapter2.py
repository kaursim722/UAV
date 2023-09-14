"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter2.py (from the root directory) -or-
python testChapter2.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:

import math

import sys


sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-5)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-4], [0], [-1]]))
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


#%% dcm2Euler():
print("Beginning testing of Rotations.dcm2Euler()")

cur_test = "dcm2Euler test 1"
R = Rotations.dcm2Euler([[0.98480, 0.17364, 0], [-0.17364, 0.98480, 0], [0, 0, 1]])
orig_vector = [[R[0]], [R[1]], [R[2]]]
expected_vec = [[0.1745262180053443],[0],[0]]
if not evaluateTest(cur_test, compareVectors(expected_vec, orig_vector) ):
	print(f"{expected_vec} != {orig_vector}")

#%% dcm2Euler():
print("Beginning testing of Rotations.dcm2Euler()")

cur_test = "dcm2Euler test 2"
R = Rotations.dcm2Euler([[1,0,0], [0,1,0], [0,0,1]])
orig_vector = [[R[0]], [R[1]], [R[2]]]
expected_vec = [[0],[0],[0]]
if not evaluateTest(cur_test, compareVectors(expected_vec, orig_vector) ):
	print(f"{expected_vec} != {orig_vector}")


#%% Euler2dcm():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm  test 1"
R = Rotations.euler2DCM(0, 0, 45*math.pi/180)
expected_vec = [[1,0,0], [0,1,0], [0,0,1]]
if not evaluateTest(cur_test, compareVectors(expected_vec, R) ):
	print(f"{expected_vec} != {R}")

#%% Euler2dcm():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm  test 2"
R = Rotations.euler2DCM(10*math.pi/180, 0, 0)
expected_vec = [[0.98480,0.17364,-0],
				[-0.17364,0.98480,0],
				[0,0,1]]

if not evaluateTest(cur_test, compareVectors(expected_vec, R) ):
	print(f"{expected_vec} != {R}")



#%% ned2enu():
print("Beginning testing of Rotations.ned2enu()")

cur_test = "ned2enu test 1"
R = Rotations.ned2enu([[0,1,0], [2, 1, 3], [4,5,6]])
expected_vec = [[1, 0, 0],[1, 2, -3],[5, 4, -6]]
if not evaluateTest(cur_test, compareVectors(expected_vec, R) ):
	print(f"{expected_vec} != {R}")

#%% ned2enu():
print("Beginning testing of Rotations.ned2enu()")

cur_test = "ned2enu test 2"
R = Rotations.ned2enu([[3,5,6], [7,9,1], [2,5,2]])
expected_vec = [[5,3,-6],[9,7,-1],[5,2,-2]]
if not evaluateTest(cur_test, compareVectors(expected_vec, R) ):
	print(f"{expected_vec} != {R}")


#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]

