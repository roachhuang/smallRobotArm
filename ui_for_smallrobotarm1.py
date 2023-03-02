import tkinter as tk
from time import sleep
import init_serial as com

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
        self.ser = com.init_ser()
        # cannot just send right after init_ser
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

    def send_command(self):
        # read the joint angles from the sliders
        joint=[0,0,0,0,0,0]
        for i in range(6):
            joint[i] = float(self.sliders[i].get())

        # construct the command string
        command = "g{},{},{},{},{},{}\n".format(
            joint[0], joint[1], joint[2], joint[3], joint[4], joint[5])
        print(command)
        # send the command to the Arduino
        self.ser.write(command.encode('utf-8'))

    def create_widgets(self):
        # create labels for the joint sliders
        labels = ["Joint 1", "Joint 2", "Joint 3",
                  "Joint 4", "Joint 5", "Joint 6"]
        for i in range(len(labels)):
            joint_label = tk.Label(self, text=labels[i])
            joint_label.grid(row=i, column=0)

        # create sliders for the joint angles
        self.sliders = []
        for i in range(6):
            joint_slider = tk.Scale(
                self, from_=-180, to=180, orient=tk.HORIZONTAL, length=200)
            joint_slider.grid(row=i, column=1)
            self.sliders.append(joint_slider)

        # create buttons to adjust slider values
        buttons = ["-", "+"]
        for i in range(2):
            for j in range(6):
                joint_button = tk.Button(
                    self, text=buttons[i],
                    command=lambda slider=self.sliders[j], inc=10*(-1)**i:
                        self.adjust_slider_value(slider, -inc)
                )
                joint_button.grid(row=j, column=i+2, padx=10)

        # create a button to send the command to the Arduino
        send_button = tk.Button(self, text="Send Command",
                                command=self.send_command)
        send_button.grid(row=6, column=0, columnspan=4)



if __name__ == "__main__":
    root = tk.Tk()
    app = Application(master=root)
    app.mainloop()
