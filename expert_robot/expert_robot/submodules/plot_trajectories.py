import numpy
import matplotlib.pyplot as plt
import pandas as pd

# read in csv file as a data frame
df = pd.read_csv('expert_robot/trajectories/tool_pose_20220323_155512.csv')

# Extract the x, y, and z reaches depending on the current dimension
x_reaches = df[df['current_dim'] == 'x']
y_reaches = df[df['current_dim'] == 'y']
z_reaches = df[df['current_dim'] == 'z']

# Plot the x, y, and z reaches in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_reaches['position_x'], x_reaches['position_y'], x_reaches['position_z'], label='x')
ax.plot(y_reaches['position_x'], y_reaches['position_y'], y_reaches['position_z'], label='y')
ax.plot(z_reaches['position_x'], z_reaches['position_y'], z_reaches['position_z'], label='z')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()


