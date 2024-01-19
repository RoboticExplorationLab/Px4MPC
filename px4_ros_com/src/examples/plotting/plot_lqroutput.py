import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# Parse command line arguments, read two files locations
parser = argparse.ArgumentParser()
parser.add_argument("file", help="File to plot")
parser.add_argument("file2", help="File to plot")
args = parser.parse_args()


# Check if file exists
if not os.path.isfile(args.file):
    print("File not found")
    sys.exit()

if not os.path.isfile(args.file2):
    print("File not found")
    sys.exit() 

# Read file
data = np.loadtxt(args.file)
data = data.reshape(-1,1)
data2 = np.loadtxt(args.file2)
data2 = data2.reshape(-1,1)

length = data.shape[0]
no_of_data_points = int(length/4)
xaxis = np.linspace(0, no_of_data_points, no_of_data_points)
length2 = data2.shape[0]
no_of_data_points2 = int(length2/4)
xaxis2 = np.linspace(0, no_of_data_points2, no_of_data_points2)

# Plot data
plt.figure()
plt.xlabel('Time step')
plt.ylabel('motor input')
U1 = data[0:(length+1):4,0]
U2 = data[1:(length+1):4,0]
U3 = data[2:(length+1):4,0]
U4 = data[3:(length+1):4,0]

plt.plot(xaxis, U1, label='U1')
plt.plot(xaxis, U2, label='U2')
plt.plot(xaxis, U3, label='U3')
plt.plot(xaxis, U4, label='U4')
plt.legend(['U1', 'U2', 'U3', 'U4'])
plt.savefig('motor_input.png')
plt.show()

# Plot data2
plt.figure()
plt.xlabel('Time step')
plt.ylabel('motor norm input')
U1 = data2[0:(length2+1):4,0]  
U2 = data2[1:(length2+1):4,0]
U3 = data2[2:(length2+1):4,0]
U4 = data2[3:(length2+1):4,0]

plt.plot(xaxis2, U1, label='U1')
plt.plot(xaxis2, U2, label='U2')
plt.plot(xaxis2, U3, label='U3')
plt.plot(xaxis2, U4, label='U4')
plt.legend(['U1', 'U2', 'U3', 'U4'])
plt.savefig('motor_norm_input.png')
plt.show()
