import numpy as np
import pandas as pd
import csv
from matplotlib import pyplot as plt

# grid
plt.xlim((-40,40))
plt.ylim((-40,40))
plt.plot([-40,40],[0,0], linewidth = 1, color = "black")
plt.plot([0,0],[40,-40], linewidth = 1, color = "black")

# read coords from file
x = []
y = []

with open("GPS_Points.txt", "r") as f:
	reader = csv.reader(f, delimiter = " ")
	for i in reader:
		x.append(i[0])
		y.append(i[1])

print(x,y)

#plot points and trace path
new_x = []
new_y = []

for i in range(len(x)):
	new_x.append(x[i])
	new_y.append(y[i])
	new_x[i] = float(new_x[i])
	new_y[i] = float(new_y[i])
	plt.scatter(new_x[i],new_y[i], color = 'b', marker = 'o')
	plt.plot(new_x,new_y, color = 'b')
	plt.pause(0.1)

plt.show()