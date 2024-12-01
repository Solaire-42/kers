import os
import math
import numpy as np
import matplotlib.pyplot as plt

DIAMETER_FRONT = 0.49  # diameter [m]
DIAMETER_BACK = 0.67  # diameter [m]
GEAR_RATIO = 4.4

u_front = DIAMETER_FRONT * math.pi  # circumference [m]
u_back = DIAMETER_BACK * math.pi  # circumference [m]

v = list(range(0, 10, 1))  # velocities of interests [m/s]

rpm_front = []  # Empty list to fill
rpm_back = []  # Empty list to fill

for i in v:
    rpm_front.append((i/u_front)*60)  # rpm = velocity/circumference * 60
    rpm_back.append((i/u_back)*60)  # rpm = velocity/circumference * 60

v_kmh = [value * 3.6 for value in v]  # velocity in km/h

# Plot velocity to wheel rpm
fig = plt.figure(1)
axis_1 = fig.add_subplot(111)
axis_1.plot(rpm_front, v_kmh)
axis_1.plot(rpm_back, v_kmh)
axis_1.set_xlabel('Mechanical speed [RPM]', fontsize='12')
axis_1.set_ylabel('v [km/h]', fontsize='12')
axis_1.set_title('Velocity per RPM', fontsize='14')
axis_1.legend((f'Front wheel (d={DIAMETER_FRONT}m)', f'Back wheel (d={DIAMETER_BACK}m)'), loc='upper left')
axis_1.grid()

# Add x-axis for electrical rotational speed
electrical_rotational_speed_xaxis_limit = int(axis_1.get_xlim()[1] * GEAR_RATIO)

# Scale function and it's inverse to scale the second x-axis to the first x-axis based on gear ratio (x-axis mechanical rotation -> x-axis electrical rotation)
def scale_gear_ratio(x):
    return x * GEAR_RATIO

def inv_scale_gear_ratio(x):
    return x / GEAR_RATIO

axis_2 = axis_1.secondary_xaxis(-0.2, functions=(scale_gear_ratio, inv_scale_gear_ratio))  # Scale x-axis 2 based on x-axis 1
axis_2.set_xticks(axis_1.get_xticks() * GEAR_RATIO)  # Set ticks based on x-axis 1 and gear ratio
axis_2.set_xlabel(f'Electrical speed (gear ratio: 1/{GEAR_RATIO}) [RPM]', fontsize='12')

plt.tight_layout()

# Save figure
figure = plt.gcf()

# Change to current directory and save file
name_of_current_directory = os.path.dirname(__file__)
os.chdir(name_of_current_directory)
figure.savefig('velocity_per_rpm.png')

plt.show()
