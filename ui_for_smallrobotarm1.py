import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
# from roboticstoolbox.backends.PyPlot import PyPlot

import tkinter as tk
from time import sleep
import init_serial as com


class Application(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.prevInputVal = np.zeros(6)
        self.fromAngles = np.array([0.0, -78.51, 73.9, 0.0, -90.0, 0.0])
        self.currJoint =np.zeros(6)
        self.master = master
        self.master.title("Robot Arm Control Panel")
        self.master.geometry("1280x1024")
        self.master.resizable(False, False)
        self.pack()
        self.create_widgets()

        # initialize the serial connection to the Arduino
        self.ser = com.init_ser()
        # there must be a dealy here!!!
        sleep(1)
        self.ser.write(b"en\n")
        sleep(.5)
        self.ser.write(b"rst\n")
        sleep(.5)

    def adjust_slider_value(self, slider, increment):
        """
        Adjust the value of the given slider by the given increment.

        Args:
            slider: The slider to adjust.
            increment: The amount to increment the slider value by.
        """
        current_value = slider.get()
        new_value = current_value + increment
        slider.set(new_value)

    def animate(self, q):
        # example of joint angle update
        # q = frame / 100 * np.array([np.pi/2, np.pi/4, np.pi/2, 0, 0, 0])
        robot.plot(q, fig=fig, backend="pyplot")
        return ax.collections

    def reset_command(self):
        self.fromAngles = [0.0, -78.51, 73.9, 0.0, -90.0, 0.0]
        dJoint = self.fromAngles - self.currJoint
        command = "g{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
            *dJoint)
        print(command)
        # send the command to the Arduino
        self.ser.write(command.encode('utf-8'))
        self.prevInputVal = np.zeros(6)
        self.currJoint = np.zeros(6)
        for i in range(6):
            self.sliders[i].set(0)
        robot.plot(np.radians(self.fromAngles), fig=fig, backend="PyPlot")
        self.currJoint=self.fromAngles
        
    def send_command(self):
        # read the joint angles from the sliders
        currInputVal=np.zeros(6)

        for i in range(6):
            currInputVal[i] = float(self.sliders[i].get())

        dJoint = currInputVal - self.prevInputVal
        self.prevInputVal = currInputVal

        self.currJoint = self.fromAngles+ dJoint
        self.fromAngles = self.currJoint

        # construct the command string
        anim = FuncAnimation(fig, self.animate(np.radians(self.currJoint)))
        plt.show()

        command = "g{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(*dJoint)
        print(command)
        # send the command to the Arduino
        self.ser.write(command.encode('utf-8'))

    def create_widgets(self):
        # create labels for the joint sliders
        labels = ["Joint 1", "Joint 2", "Joint 3",
                  "Joint 4", "Joint 5", "Joint 6"]
        for i in range(len(labels)):
            joint_label = tk.Label(self, text=labels[i], font=("Arial", 24))
            joint_label.grid(row=i, column=0)

        # create sliders for the joint angles
        self.sliders = []
        for i in range(6):
            joint_slider = tk.Scale(
                self, from_=-180, to=180, orient=tk.HORIZONTAL, length=800, font=("Arial", 18))
            joint_slider.grid(row=i, column=1)
            self.sliders.append(joint_slider)

        # create buttons to adjust slider values
        buttons = ["-", "+"]
        for i in range(2):
            for j in range(6):
                joint_button = tk.Button(
                    self, text=buttons[i],
                    command=lambda slider=self.sliders[j], inc=10*(-1)**i:
                        self.adjust_slider_value(slider, -inc), width=8, height=2, font=("Arial", 18)
                )
                joint_button.grid(row=j, column=i+2, padx=10)

        # create a button to send the command to the Arduino
        send_button = tk.Button(self, text="Send Command",
                                command=self.send_command, width=16, height=2,font=("Arial", 24))
        send_button.grid(row=6, column=0, columnspan=4)

        reset_button = tk.Button(self, text="Reset",
                                 command=self.reset_command, width=16, height=2, font=("Arial", 24))
        reset_button.grid(row=6, column=2, columnspan=4)


if __name__ == "__main__":
    a1, a2, a3 = 47.0, 110.0, 26.0
    d1, d4, d6 = 133.0, 117.50, 28.0

    dh_table = [
        RevoluteDH(d=d1, a=a1, alpha=-np.pi/2),    # joint 1
        RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi/2),             # joint 2
        RevoluteDH(d=0, a=a3, alpha=-np.pi/2),      # joint 3
        RevoluteDH(d=d4, a=0, alpha=np.pi/2),       # joint 4
        RevoluteDH(d=0, a=0, alpha=-np.pi/2),       # joint 5
        RevoluteDH(d=d6, a=0, alpha=0)              # joint 6
    ]
    frames=[
        SE3.Tx(0)*SE3.Ty(0)*SE3.Tz(20),
        SE3.Rz(np.pi)*SE3.Ry(-np.pi/2)*SE3.Rz(0)
    ]

    robot = DHRobot(dh_table, name='SmallRobotArm', base=frames[0],tool=frames[-1])
    fig, ax = plt.subplots()
    robot.plot([0, np.radians(-78.51), np.radians(73.9),
               0, -np.pi/2, 0], fig=fig, backend="pyplot")
    root = tk.Tk()
    app = Application(master=root)
    app.mainloop()
