import math
import matplotlib.pyplot as plt

d_front = 0.49  # diameter [m]
d_back = 0.67  # diameter [m]

u_front = d_front * math.pi  # circumference [m]
u_back = d_back * math.pi  # circumference [m]

v = list(range(0, 10, 1))  # velocities of interests [m/s]

rpm_front = []  # Empty list to fill
rpm_back = []  # Empty list to fill

for i in v:
    rpm_front.append((i/u_front)*60)  # rpm = velocity/circumference * 60
    rpm_back.append((i/u_back)*60)  # rpm = velocity/circumference * 60

v_kmh = [value * 3.6 for value in v]  # velocity in km/h

fig = plt.figure(1)
plt.plot(rpm_front, v_kmh)
plt.plot(rpm_back, v_kmh)
plt.title("Velocity per RPM", fontsize='16')
plt.xlabel("RPM",fontsize='13')
plt.ylabel("v [km/h]",fontsize='13')
plt.legend(('Front', 'Back'), loc='best')
plt.grid()
plt.show()
