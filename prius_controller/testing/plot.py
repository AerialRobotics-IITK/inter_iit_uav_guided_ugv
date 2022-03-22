from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas

points = pandas.read_csv('/home/akshit/ros/drdo22_ws/src/inter_iit_uav_guided_ugv/prius_controller/testing/mean_path/world2/world2_in.csv')
out = pandas.read_csv('/home/akshit/ros/drdo22_ws/src/inter_iit_uav_guided_ugv/prius_controller/testing/mean_path/world2/world2_out.csv')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x = points['x'].values
y = points['y'].values
z = points['z'].values

x_o = out['x'].values
y_o = out['y'].values
z_o = out['z'].values

#  x   y   z  
# 1.1,1.2,1.3
# 2.1,2.2,2.3
# 3.1,3.2,3.3
# 4.1,4.2,4.3

ax.scatter(x, y, z, c='r', marker='o')
ax.scatter(x_o, y_o, z_o, c='b', marker='o')

plt.show()