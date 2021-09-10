import numpy as np
import matplotlib.pyplot as plt
import sys


def smooth_with_moving_average(arr, ma):
    arr_temp = arr.copy()
    for r in range(ma,len(arr_temp)):
        ma_sum = 0.0
        ma_count = 0

        for i in range(r - ma, r):
            ma_sum += arr_temp[i]
            ma_count += 1.0
        arr[r] = ma_sum / ma_count

    return arr


date = sys.argv[1]
print(date)
file = open("../data/"+date+".txt", "r")

line_list = []


for line in file:
    stripped_line = line.strip()
    curr_pos_list = stripped_line.split(', ')

    if curr_pos_list[0] != 'None':
        num_list = [float(i) for i in curr_pos_list]
        line_list.append(num_list)


line_list = np.array(line_list)
#print(line_list)
line_list = line_list[:, :]

fig, axs = plt.subplots(6)

t_step = line_list[:,0]
z_pos = line_list[:,1]
z_vel = line_list[:,2]
rot = line_list[:,3]
rot_vel = line_list[:,4]


#smooth out yoyo pos velocity
for z in range(1, len(z_vel) - 1):
    if abs(z_vel[z - 1] - z_vel[z]) > 2.0:
        z_vel[z] = z_vel[z-1]#(z_vel[z-1] + z_vel[z+1])/2


z_vel = smooth_with_moving_average(z_vel, 5)



#smooth out rotation velocity
for r in range(1,len(rot_vel)-1):
    if rot_vel[r] > 300 or rot_vel[r] < -300:
        rot_vel[r] = (rot_vel[r-1] + rot_vel[r+1])/2.0

for r in range(1,len(rot_vel)-1):
    if rot_vel[r] > 300 or rot_vel[r] < -300:
        rot_vel[r] = (rot_vel[r-1] + rot_vel[r+1])/2.0


rot_vel = smooth_with_moving_average(rot_vel, 10)





ee_pos = line_list[:,5]
vel_input = line_list[:,6]

axs[0].plot(t_step, z_pos)
axs[0].set_ylim([0.0,1.0])
axs[0].invert_yaxis()
axs[0].set_title('yoyo z pos')

axs[1].plot(t_step, z_vel)
axs[1].set_title('yoyo z vel')

axs[2].plot(t_step, rot)
axs[2].set_title('yoyo rotation')

axs[3].plot(t_step, rot_vel)
axs[3].set_title('yoyo rot vel')

axs[4].plot(t_step, ee_pos)
axs[4].set_title('ee z pos')

axs[5].plot(t_step, vel_input)
axs[5].set_title('input velocity')


plt.show()


file.close()