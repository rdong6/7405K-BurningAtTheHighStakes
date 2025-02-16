import matplotlib.pyplot as plt
import sys
from dataclasses import dataclass


@dataclass
class Path:
    x: list[float]
    y: list[float]
    sizes: list[float]


@dataclass
class Trajectory:
    t: list[float]
    x: list[float]
    y: list[float]
    vel: list[float]
    rightWheelVel: list[float]
    leftWheelVel: list[float]
    accel: list[float]
    curvature: list[float]
    sizes: list[float]


trajectories: list[Trajectory] = []
trackwidth = 11.5

with open(sys.argv[1], "r") as file:
    t: list[float] = []
    x: list[float] = []
    y: list[float] = []
    vel: list[float] = []
    rightWheelVel: list[float] = []
    leftWheelVel: list[float] = []
    accel: list[float] = []
    curvature: list[float] = []
    sizes: list[float] = []
    for line in file:
        tokens = line.split(" ")
        curVel = float(tokens[3])
        curCurv = float(tokens[5])
        curAngularVel = curVel * curCurv

        t.append(float(tokens[0]))
        x.append(float(tokens[1]))
        y.append(float(tokens[2]))
        vel.append(curVel)
        rightWheelVel.append(curVel + curAngularVel * trackwidth * 0.5)
        leftWheelVel.append(curVel - curAngularVel * trackwidth * 0.5)
        accel.append(float(tokens[4]))
        curvature.append(float(tokens[5]))
        sizes.append(5)
        tokens = line.split(" ")

    trajectories.append(Trajectory(t, x, y, vel, rightWheelVel, leftWheelVel, accel, curvature, sizes))

trajectory = trajectories[0]
plt.figure()
plt.title("Position")
plt.xlabel("X (in)")
plt.ylabel("Y (in)")
plt.scatter(trajectory.x, trajectory.y, trajectory.sizes)

plt.figure()
plt.title("Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Vel (in/s)")
# plt.plot(trajectory.t, trajectory.vel, 'o', linewidth=2)
plt.plot(trajectory.t, trajectory.rightWheelVel, 'r', linewidth=2)
# plt.plot(trajectory.t, trajectory.leftWheelVel, 'b', linewidth=2)

plt.figure()
plt.title("Acceleration")
plt.xlabel("Time (s)")
plt.ylabel("Accel (in/s^2)")
plt.plot(trajectory.t, trajectory.accel, linewidth=2)

plt.figure()
plt.title("Curvature")
plt.xlabel("Time (s)")
plt.ylabel("Curv (1/in)")
plt.plot(trajectory.t, trajectory.curvature, linewidth=2)


plt.show()

# for path in paths:
#     plt.figure()
#     plt.plot(path.x, path.y)
#     plt.scatter(path.x, path.y, path.sizes)
#     plt.xlabel("X (in)")
#     plt.ylabel("Y (in)")
# plt.show()
