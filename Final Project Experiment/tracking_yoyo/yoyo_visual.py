import numpy as np
import matplotlib.pyplot as plt

file = open("yoyo_pos.txt", "r")
line_list = []


for line in file:
    stripped_line = line.strip()
    curr_pos_list = stripped_line.split(', ')

    if curr_pos_list[0] != 'None':
        int_list = [int(i) for i in curr_pos_list]
        line_list.append(int_list)


line_list = np.array(line_list)
#print(line_list)

x_pos = line_list[:,0]
y_pos = line_list[:,1]


plt.plot(x_pos, y_pos)
plt.xlim([0, 750])
plt.ylim([0, 540])
plt.show()
file.close()