import numpy as np
import matplotlib.pyplot as plt

class Unicycle:
    def __init__(self, x: float, y: float, theta: float, dt: float):
        self.x = x
        self.y = y
        self.theta = theta
        self.dt = dt

        # Store the points of the trajectory to plot
        self.x_points = [self.x]
        self.y_points = [self.y]

    def step(self, v: float, w: float, n: int):
        x = 0
        y = 0
        theta = self.theta
        for i in range(n):
            x = x + v*(self.dt)*(np.cos(theta))
            y = y + v*(self.dt)*(np.sin(theta))
            theta = theta + w*(self.dt)
            self.x_points.append(x)
            self.y_points.append(y)

        return x, y, theta

    def plot(self, v: float, w: float):
        plt.title(f"Unicycle Model: {v}, {w}")
        plt.xlabel("X-Coordinates")
        plt.ylabel("Y-Coordinates")
        plt.plot(self.x_points, self.y_points, color="red", alpha=0.75)
        plt.grid()

        # If you want to view the plot uncomment plt.show() and comment out plt.savefig()
        plt.show()
        # If you want to save the file, uncomment plt.savefig() and comment out plt.show()
        #plt.savefig(f"Unicycle_{v}_{w}.png")

if __name__ == "__main__":
    print("Unicycle Model Assignment")
    robot1 = Unicycle(0,0,0,0.1)
    final_x,final_y,final_theta = robot1.step(1,0.5,25)
    print("Final pose (x,y,theta):",final_x,final_y,final_theta)
    robot1.plot(1,0.5)

    robot2 = Unicycle(0,0,1.57,0.2)
    final_x,final_y,final_theta = robot2.step(0.5,1,10)
    print("Final pose (x,y,theta):",final_x,final_y,final_theta)
    robot2.plot(0.5,1)

    robot3 = Unicycle(0,0,0.77,0.05)
    final_x,final_y,final_theta = robot3.step(5,4,50)
    print("Final pose (x,y,theta):",final_x,final_y,final_theta)
    robot3.plot(5,4)
