import tkinter as tk
from tkinter import filedialog
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Waypoints and their labels
waypoints = np.array([[2.95, 0.0], [3.006423, 1.83], [1.44830, 2.118003], [1.443652, 3.85923]])
labels = ['door', 'toilet', 'mirror', 'television']

def upload_file(file_title, plot_title):
    filename = filedialog.askopenfilename(initialdir="/home/raoui/ros21_ws/src/nav2_new_features/scripts/experiences/2024_01_15_11_41_44", title=file_title, filetypes=(("text files","*.txt"),("all files","*.*")))
    print('Selected:', filename)

    # Read the file into a numpy array
    data = np.loadtxt(filename)
    transformations = data.reshape((-1, 3, 4))
    positions = transformations[:, :, 3]

    # Calculate the minimum and maximum values for the x and y coordinates
    min_x = min(np.min(positions[:,0]), np.min(waypoints[:,0]))
    max_x = max(np.max(positions[:,0]), np.max(waypoints[:,0]))
    min_y = min(np.min(positions[:,1]), np.min(waypoints[:,1]))
    max_y = max(np.max(positions[:,1]), np.max(waypoints[:,1]))

    # Plot the array
    fig, ax = plt.subplots()
    ax.plot(positions[:,0], positions[:,1])
    for i in range(len(waypoints)):
        ax.scatter(waypoints[i,0], waypoints[i,1], color='red', marker='o')  # Add waypoint
        ax.annotate(labels[i], (waypoints[i,0], waypoints[i,1]))  # Add label
    ax.set_title(plot_title)
    ax.set_xlim(min_x, max_x)  # Set limits of x-axis
    ax.set_ylim(min_y, max_y)  # Set limits of y-axis
    ax.set_aspect('equal', 'box')  # Set aspect ratio to be equal
    # Display the plot on the interface
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.draw()
    canvas.get_tk_widget().pack()

root = tk.Tk()
root.geometry("1200x1000")  # Set initial size of the window

button1 = tk.Button(root, text="Upload Ground Truth", command=lambda: upload_file("Select poses.txt", "AMCL localization"))
button1.pack(side='right', anchor='ne')

button2 = tk.Button(root, text="Upload Predictions", command=lambda: upload_file("Select poses_from_odometer.txt", "Odometry localization"))
button2.pack(side='right', anchor='ne')

# Add a button to close the interface
button3 = tk.Button(root, text="Close", command=root.destroy)
button3.pack(side='right', anchor='ne')

root.mainloop()