import numpy as np

import matplotlib.pyplot as plt

x_list = []
y_list = []

file1 = open("pos_data_x.txt", "r")
for line in  file1:
	#x_list.append(float(file1.readline(line)))
	x_list.append(float(line))
	
file2 = open("pos_data_y.txt", "r")
for line in  file2:
	#y_list.append(float(file1.readline(line)))
	y_list.append(float(line))
	

file2.close
file1.close

desired_x = [0.0, 0.5, 0.5]
desired_y = [0.0, 0.0, 0.5]

desired_pos_x_list = [0.5, 0.5, 0.0, 0.0]
desired_pos_y_list = [0.0, 0.5, 0.5, 0.0]

plt.plot(x_list, y_list, label = 'robot trajectory')
plt.plot(desired_x, desired_y, marker = 'x', label = 'desired position')
#plt.plot(desired_pos_x_list, desired_pos_y_list, marker = 'x', label = 'desired position')
plt.title("Robot trajectory based on the wheel encoders ")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
#plt.axis("square")
plt.legend()
plt.show()
