
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
# from roboticstoolbox.backends.PyPlot import PyPlot
import robotarm_class as smRbt
import tkinter as tk
from time import sleep
import init_serial as com

font_size = 18
slider_size = 14
# power off pose
REST_ANGLES = np.array([0.0, -78.51, 73.9, 0.0, -90.0, 0.0])


def get_coord(x, y):
    # store the current mousebutton
    b = ax.button_pressed
    # set current mousebutton to something unreasonable
    ax.button_pressed = -1
    # get the coordinate string out
    s = ax.format_coord(x, y)
    # set the mousebutton back to its previous state
    ax.button_pressed = b
    return s


class Application(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.__prevInputVal = np.zeros(6)
        self.__from_deg = REST_ANGLES
        self.__to_deg = np.zeros(6)
        self.__d_deg = 0
        self.master = master
        self.master.title("Robot Arm Control Panel")
        self.master.geometry("1024x768")
        self.master.resizable(True, True)
        self.pack()
        self.create_widgets()

        # initialize the serial connection to the Arduin
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

    def on_move(self, event):
        # if event.inaxes == ax:
        str_xyz = get_coord(event.xdata, event.ydata).rstrip()
        list_xyz = str_xyz.split(',')
        num_xyz = [float(value) for key, value in [pair.split('=')
                                                   for pair in list_xyz]]
        pose = np.resize(num_xyz, 6)
        pose[3:6] = self.__from_deg[3:6]
        print(pose)
        # j in deg
        # j = sm_rbt_arm.ik(pose)
        T = SE3(pose[0],  pose[1], pose[2]) * SE3.Rz(np.radians(pose[3])
                                                     ) * SE3.Ry(np.radians(pose[4])) * SE3.Rz(np.radians(pose[5]))
        j = robot.ikine_LM(T)
        q = np.degrees(j.q)

        for i in range(6):
            self.sliders[i].set(q[i])
        self.send_command()

    def animate(self, frame):
        # example of joint angle update
        # q = frame / 100 * np.array([np.pi/2, np.pi/4, np.pi/2, 0, 0, 0])
        q = np.radians(self.__from_deg + self.__d_deg/frame)
        robot.plot(q, fig=fig)  # , fig=fig, backend="pyplot")

        # animate is async, so we need to do this at here
        if frame == 1:
            self.__from_deg = self.__to_deg
        # robot.plot(q, fig=fig, backend="pyplot")
        return ax.collections

    def reset_command(self):
        for i in range(6):
            self.sliders[i].set(0)
        self.send_command()

        '''
        self.__from_deg = REST_ANGLES
        self.__d_deg = self.__from_deg - self.__to_deg
        command = "g{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
            *self.__d_deg)
        print(command)
        # send the command to the Arduino
        self.ser.write(command.encode('utf-8'))
        self.__prevInputVal = np.zeros(6)
        # self.currJoint = np.zeros(6)
        for i in range(6):
            self.sliders[i].set(0)
        robot.plot(np.radians(self.__from_deg))  # , fig=fig, backend="PyPlot")
        self.__to_deg = self.__from_deg
        '''

    def send_command(self):
        # read the joint angles from the sliders
        currInputVal = np.zeros(6)

        for i in range(6):
            currInputVal[i] = float(self.sliders[i].get())

        self.__d_deg = currInputVal - self.__prevInputVal
        self.__prevInputVal = currInputVal
        self.__to_deg = self.__from_deg + self.__d_deg

        # construct the command string, delay of 100ms btw each frame
        anim = FuncAnimation(fig, self.animate, frames=range(
            5, 0, -1), interval=20, blit=True, repeat=False)
        #anim = FuncAnimation(fig, self.animate, frames=range(10), interval=200, blit=True, repeat=False)

        # plt.show()

        command = "g{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
            *self.__d_deg)
        print(command)
        # send the command to the Arduino
        self.ser.write(command.encode('utf-8'))

    def create_widgets(self):
        # create labels for the joint sliders
        labels = ["Joint 1", "Joint 2", "Joint 3",
                  "Joint 4", "Joint 5", "Joint 6"]
        for i in range(len(labels)):
            joint_label = tk.Label(
                self, text=labels[i], font=("Arial", font_size))
            joint_label.grid(row=i, column=0)

        # create sliders for the joint angles
        self.sliders = []
        for i in range(6):
            joint_slider = tk.Scale(
                self, from_=-180, to=180, orient=tk.HORIZONTAL, length=800, font=("Arial", slider_size))
            joint_slider.grid(row=i, column=1)
            self.sliders.append(joint_slider)

        # create buttons to adjust slider values
        buttons = ["-", "+"]
        for i in range(2):
            for j in range(6):
                joint_button = tk.Button(
                    self, text=buttons[i],
                    command=lambda slider=self.sliders[j], inc=10*(-1)**i:
                        self.adjust_slider_value(slider, -inc), width=8, height=2, font=("Arial", slider_size)
                )
                joint_button.grid(row=j, column=i+2, padx=10)

        # create a button to send the command to the Arduino
        send_button = tk.Button(self, text="Send Command",
                                command=self.send_command, width=16, height=2, font=("Arial", font_size))
        send_button.grid(row=6, column=0, columnspan=4)

        reset_button = tk.Button(self, text="Reset",
                                 command=self.reset_command, width=16, height=2, font=("Arial", font_size))
        reset_button.grid(row=6, column=2, columnspan=4)


if __name__ == "__main__":
    a1, a2, a3 = 47.0, 110.0, 26.0
    d1, d4, d6 = 133.0, 117.50, 28.0

    dh_params = [
        RevoluteDH(d=d1, a=a1, alpha=-np.pi/2),    # joint 1
        RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi/2),             # joint 2
        RevoluteDH(d=0, a=a3, alpha=-np.pi/2),      # joint 3
        RevoluteDH(d=d4, a=0, alpha=np.pi/2),       # joint 4
        RevoluteDH(d=0, a=0, alpha=-np.pi/2),       # joint 5
        RevoluteDH(d=d6, a=0, alpha=0),              # joint 6
    ]

    frames = [
        SE3.Tx(0)*SE3.Ty(0)*SE3.Tz(20),
        SE3.Rz(np.pi)*SE3.Ry(-np.pi/2)*SE3.Rz(0)
    ]
    sm_rbt_arm = smRbt.RobotArm(6, dh_params)

    # , base=frames[0],tool=frames[-1])
    robot = DHRobot(dh_params, name='SmallRobotArm')
    # fig, ax = plt.subplots()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-200, 200)
    ax.set_ylim(-200, 200)
    ax.set_zlim(0, 200)

    robot.plot([0, np.radians(-78.51), np.radians(73.9),
               0, -np.pi/2, 0], fig=fig, backend="pyplot")

    root = tk.Tk()
    # root.resizable(True, True)
    app = Application(master=root)
    fig.canvas.callbacks.connect('button_press_event', app.on_move)
    app.mainloop()
