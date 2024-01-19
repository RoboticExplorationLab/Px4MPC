import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# Parse command line arguments, read file location
parser = argparse.ArgumentParser()
parser.add_argument("file", help="File to plot")
args = parser.parse_args()

# Check if file exists
if not os.path.isfile(args.file):
    print("File not found")
    sys.exit()

# Read file
data = np.loadtxt(args.file)
shape = data.shape  
print(shape)
rows = shape[0]
columns = shape[1]

# Plot columns in same figure
plt.figure()
plt.xlabel('Time step')
plt.ylabel('state error')
plt.title('State error')
plt.plot(data[:,0], label='x')
plt.plot(data[:,1], label='y')
plt.plot(data[:,2], label='z')
plt.plot(data[:,3], label='phi')
plt.plot(data[:,4], label='theta')
plt.plot(data[:,5], label='psi')
plt.plot(data[:,6], label='x_dot')
plt.plot(data[:,7], label='y_dot')
plt.plot(data[:,8], label='z_dot')
plt.plot(data[:,9], label='phi_dot')
plt.plot(data[:,10], label='theta_dot')
plt.plot(data[:,11], label='psi_dot')
plt.legend(['x', 'y', 'z', 'phi', 'theta', 'psi', 'x_dot', 'y_dot', 'z_dot', 'phi_dot', 'theta_dot', 'psi_dot'])
plt.savefig('state_error.png')
plt.show()

# Plot columns in separate subplots
plt.figure()
plt.subplot(4,3,1)
plt.plot(data[:,0])
plt.xlabel('Time step')
plt.ylabel('x')
plt.subplot(4,3,2)
plt.plot(data[:,1])
plt.xlabel('Time step')
plt.ylabel('y')
plt.subplot(4,3,3)
plt.plot(data[:,2])
plt.xlabel('Time step')
plt.ylabel('z')
plt.subplot(4,3,4)
plt.plot(data[:,3])
plt.xlabel('Time step')
plt.ylabel('phi')
plt.subplot(4,3,5)
plt.plot(data[:,4])
plt.xlabel('Time step')
plt.ylabel('theta')
plt.subplot(4,3,6)
plt.plot(data[:,5])
plt.xlabel('Time step')
plt.ylabel('psi')
plt.subplot(4,3,7)
plt.plot(data[:,6])
plt.xlabel('Time step')
plt.ylabel('x_dot')
plt.subplot(4,3,8)
plt.plot(data[:,7])
plt.xlabel('Time step')
plt.ylabel('y_dot')
plt.subplot(4,3,9)
plt.plot(data[:,8])
plt.xlabel('Time step')
plt.ylabel('z_dot')
plt.subplot(4,3,10)
plt.plot(data[:,9])
plt.xlabel('Time step')
plt.ylabel('phi_dot')
plt.subplot(4,3,11)
plt.plot(data[:,10])
plt.xlabel('Time step')
plt.ylabel('theta_dot')
plt.subplot(4,3,12)
plt.plot(data[:,11])
plt.xlabel('Time step')
plt.ylabel('psi_dot')
plt.savefig('state_error_subplots.png')
plt.show()

