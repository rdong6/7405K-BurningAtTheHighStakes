import matplotlib.pyplot as plt
import sys
from dataclasses import dataclass


@dataclass
class Path:
    x: list[float]
    y: list[float]
    sizes: list[float]


paths: list[Path] = []

for i in range(len(sys.argv) - 1):
    with open(sys.argv[i + 1], "r") as file:
        pointsX = []
        pointsY = []
        sizes = []  # size of each point
        for line in file:
            tokens = line.split(" ")
            pointsX.append(float(tokens[0]))
            pointsY.append(float(tokens[1]))
            sizes.append(5)
        paths.append(Path(pointsX, pointsY, sizes))

for path in paths:
    plt.figure()
    plt.plot(path.x, path.y)
    plt.scatter(path.x, path.y, path.sizes)
    plt.xlabel("X (in)")
    plt.ylabel("Y (in)")
plt.show()
