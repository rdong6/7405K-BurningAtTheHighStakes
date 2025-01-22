import matplotlib.pyplot as plt
import sys

t = []
pointsX = []
pointsY = []
vel = []
sizes = [] # size of each point
with open(sys.argv[1], "r") as file:
    for line in file:
        tokens = line.split(" ")
        t.append(float(tokens[0]))
        pointsX.append(float(tokens[1]))
        pointsY.append(float(tokens[2]))
        vel.append(float(tokens[3]))
        sizes.append(10)

f = plt.figure()
plt.plot(pointsX, pointsY)
plt.scatter(pointsX, pointsY,sizes)
plt.xlabel("X (in)")
plt.ylabel("Y (in)")

plt.figure()
plt.plot(t, vel)
plt.xlabel("Time (s)")
plt.ylabel("Vel (in/s)")
plt.show()