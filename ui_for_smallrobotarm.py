import tkinter as tk
import serial
from time import sleep

class Application(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.master.title("Robot Arm Control Panel")
        self.master.geometry("600x400")
        self.master.resizable(False, False)
        self.pack()
        self.create_widgets()

        # initialize the serial connection to the Arduino
        self.ser = serial.Serial('COM4', 115200)
        self.ser.write(b"en\n")
        sleep(1)
    def create_widgets(self):
        # create labels for the joint sliders
        joint1_label = tk.Label(self, text="Joint 1")
        joint1_label.grid(row=0, column=0)
        joint2_label = tk.Label(self, text="Joint 2")
        joint2_label.grid(row=1, column=0)
        joint3_label = tk.Label(self, text="Joint 3")
        joint3_label.grid(row=2, column=0)
        joint4_label = tk.Label(self, text="Joint 4")
        joint4_label.grid(row=3, column=0)
        joint5_label = tk.Label(self, text="Joint 5")
        joint5_label.grid(row=4, column=0)
        joint6_label = tk.Label(self, text="Joint 6")
        joint6_label.grid(row=5, column=0)

        # create sliders for the joint angles
        self.joint1_slider = tk.Scale(
            self, from_=-180, to=180, orient=tk.HORIZONTAL, length=200)
        self.joint1_slider.grid(row=0, column=1)
        self.joint2_slider = tk.Scale(
            self, from_=-180, to=180, orient=tk.HORIZONTAL, length=200)
        self.joint2_slider.grid(row=1, column=1)
        self.joint3_slider = tk.Scale(
            self, from_=-180, to=180, orient=tk.HORIZONTAL, length=200)
        self.joint3_slider.grid(row=2, column=1)
        self.joint4_slider = tk.Scale(
            self, from_=-180, to=180, orient=tk.HORIZONTAL, length=200)
        self.joint4_slider.grid(row=3, column=1)
        self.joint5_slider = tk.Scale(
            self, from_=-180, to=180, orient=tk.HORIZONTAL, length=200)
        self.joint5_slider.grid(row=4, column=1)
        self.joint6_slider = tk.Scale(
            self, from_=-180, to=180, orient=tk.HORIZONTAL, length=200)
        self.joint6_slider.grid(row=5, column=1)

        # create labels and entry boxes for the Cartesian space coordinates
        x_label = tk.Label(self, text="X")
        x_label.grid(row=0, column=2)
        self.x_entry = tk.Entry(self)
        self.x_entry.grid(row=0, column=3)

        y_label = tk.Label(self, text="Y")
        y_label.grid(row=1, column=2)
        self.y_entry = tk.Entry(self)
        self.y_entry.grid(row=1, column=3)

        z_label = tk.Label(self, text="Z")
        z_label.grid(row=2, column=2)
        self.z_entry = tk.Entry(self)
        self.z_entry.grid(row=2, column=3)

        rx_label = tk.Label(self, text="RX")
        rx_label.grid(row=3, column=2)
        self.rx_entry = tk.Entry(self)
        self.rx_entry.grid(row=3, column=3)
        ry_label = tk.Label(self, text="RY")
        ry_label.grid(row=4, column=2)
        self.ry_entry = tk.Entry(self)
        self.ry_entry.grid(row=4, column=3)

        rz_label = tk.Label(self, text="RZ")
        rz_label.grid(row=5, column=2)
        self.rz_entry = tk.Entry(self)
        self.rz_entry.grid(row=5, column=3)

        # create a button to send the command to the Arduino
        send_button = tk.Button(self, text="Send Command",
                                command=self.send_command)
        send_button.grid(row=6, column=0, columnspan=4)

    def send_command(self):
        # read the joint angles from the sliders
        joint1 = float(self.joint1_slider.get())
        joint2 = float(self.joint2_slider.get())
        joint3 = float(self.joint3_slider.get())
        joint4 = float(self.joint4_slider.get())
        joint5 = float(self.joint5_slider.get())
        joint6 = float(self.joint6_slider.get())

        # read the Cartesian coordinates from the entry boxes
        """
        x = float(self.x_entry.get())
        y = float(self.y_entry.get())
        z = float(self.z_entry.get())
        rx = float(self.rx_entry.get())
        ry = float(self.ry_entry.get())
        rz = float(self.rz_entry.get())
        """

        # construct the command string
        command = "g{},{},{},{},{},{}\n".format(
            joint1, joint2, joint3, joint4, joint5, joint6)

        # send the command to the Arduino
        self.ser.write(command.encode())


root = tk.Tk()
app = Application(master=root)
app.mainloop()
