import numpy as np
import matplotlib.pyplot as plt
import sys

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


fig, axs = plt.subplots(3)

t_step = line_list[:,0]
z_pos = line_list[:,1]
ee_pos = line_list[:,2]
vel_input = line_list[:,3]

axs[0].plot(t_step, z_pos)
axs[0].set_ylim([0.0,1.0])
axs[0].invert_yaxis()
axs[0].set_title('yoyo z pos')


axs[1].plot(t_step, ee_pos)
axs[1].plot([0.5, 1.0])
axs[1].set_title('ee z pos')

axs[2].plot(t_step, vel_input)
axs[2].set_title('input velocity')


plt.show()


file.close()