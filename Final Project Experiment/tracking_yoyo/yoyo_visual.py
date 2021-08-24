import numpy as np
import matplotlib.pyplot as plt

file = open("yoyo_pos.txt", "r")
line_list = []


for line in file:
    stripped_line = line.strip()
    curr_pos_list = stripped_line.split(', ')

    if curr_pos_list[0] != 'None':
        line_list.append(curr_pos_list)


line_list = np.array(line_list)
print(line_list)

x_pos = line_list[:500,1]
y_pos = line_list[:500,0]
plt.plot(x_pos, y_pos)
axes = plt.gca()
axes.set_xlim(0, 720)
axes.set_ylim(0, 540)

plt.show()
file.close()