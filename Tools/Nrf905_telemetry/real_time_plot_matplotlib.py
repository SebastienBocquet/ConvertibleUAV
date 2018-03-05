import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import sys
sys.path.append("../flight_analyzer")
import telemetry_lib as lib
import toolbox as tool
import numpy as np


input_dir = 'G:'
fontsize = 10
line_len = 66
HEARTBEAT_HZ = 160
HEARTBEAT_EXPORT = 1
nb_x = 20

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

def animate(i):
	ifile = open("log", "rb")
	bidon = ifile.readlines()
	imu_x = lib.extract_var(bidon, line_len, 'sonh', nb_header_lines = 0)
	print imu_x
	xs = range(len(imu_x) - 1 - nb_x, len(imu_x)-1)
	ys = imu_x[len(imu_x) - 1 - nb_x:len(imu_x)-1]
	ax1.clear()
	ax1.plot(xs, ys)

ani = animation.FuncAnimation(fig, animate, interval = 100)
plt.show()
