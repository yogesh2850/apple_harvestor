# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Demonstration of inverse kinematics using the "ikpy" Python module."""

import sys
import tempfile
try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math
from controller import Supervisor

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')


IKPY_MAX_ITERATIONS = 4

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())

# Create the arm chain from the URDF
filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))
armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, True, True, True, False])

# Initialize the arm motors and encoders.
motors = []
for link in armChain.links:
    if 'motor' in link.name:
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(0.5)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timeStep)
        motors.append(motor)

#Use of camera ffor getting coordinates
#recognized_objects = camera.getRecognitionObjects() # Refer to the recognition section is Webots doc for more info on this
#target = recognized_objects[0].get_position() # This is the position of the target in camera coordinates

# Get the arm and target nodes.
target = supervisor.getFromDef('TARGET')
arm = supervisor.getSelf()

# target_position as an input of the IK algorithm.
target1 = [1,1,0.9]
target2 = [2,-0.5,1.65]
target3 = [3,0.5,1.2]
target4 = [1.8,1.7,1.31]
target5 = [-1.3,-1.5,1.35]
target6 = [-1.4,1.6,1.35]

# Call "ikpy" to compute the inverse kinematics of the arm.
initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
ikResults = armChain.inverse_kinematics(target4, max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

print("The angles of each joints are : ", list(map(lambda r:math.degrees(r),ikResults.tolist())))
real_frame = armChain.forward_kinematics(ikResults)
print("The transformation matrix: ")
print(real_frame)
print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target4))


# Actuate the 3 first arm motors with the IK results.
for i in range(3):
    motors[i].setPosition(ikResults[i + 1])


motors[4].setPosition(-ikResults[2] - ikResults[3])
motors[5].setPosition(ikResults[5])


import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

#Plot the current position of your arm
armChain.plot(initial_position, ax, target=ikResults) #Also, ikTarget=offset_target, sorry for any confusion

#And plot the target position of your arm
armChain.plot(ikResults, ax, target=ikResults)
matplotlib.pyplot.show()
