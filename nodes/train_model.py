import numpy as np
from koopman import Koopman
import sys
import matplotlib.pyplot as plt


def basis(state, action):
    #extra_basis = np.array([np.sin(state[2]), action*np.cos(state[2])])
    action_basis = np.array([action])
    psi = np.hstack((state, action_basis))
    return psi

print(basis(np.zeros(4), 0))


num_state = 4  #yoyo-z-pos, z-vel, rot-vel, ee-z-pos
num_basis = num_state + 1
km = Koopman(basis, num_basis, num_state)

#read data
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
line_list = line_list[3100:, :]

t_step = line_list[:,0]
z_pos = line_list[:,1]
z_vel = line_list[:,2]
rot = line_list[:,3]
rot_vel = np.abs(line_list[:,4])


#smooth out yoyo pos velocity
for z in range(1, len(z_vel) - 1):
    if abs(z_vel[z - 1] - z_vel[z]) > 1.0:
        z_vel[z] = (z_vel[z-1] + z_vel[z+1])/2


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

state_list = np.hstack((z_pos.reshape(-1,1), z_vel.reshape(-1,1), rot_vel.reshape(-1,1), ee_pos.reshape(-1,1)))

#learn
for t in range(len(t_step)-1):
    km.collect_data(state_list[t], state_list[t+1], line_list[t,6])

K = km.get_full_K()
print(np.round(K,2))


#test
predict_arr = []


K_h_T = km.get_K_h_T()
state = state_list[0]
predict_state = state.copy()
for t in range(1,len(t_step)-1):
    predict_state = (K_h_T @ basis(predict_state, line_list[t,6]).reshape(-1,1)).flatten()
    predict_arr.append(predict_state)


predict_arr = np.array(predict_arr)
print(predict_arr.shape)

fig, axs = plt.subplots(num_state)
for i in range(num_state):
    if i == 0:
        #axs[i].set_ylim([0.0,1.0])
        axs[i].invert_yaxis()
    axs[i].plot(range(len(predict_arr)), predict_arr[:,i])
    axs[i].plot(range(len(state_list)), state_list[:,i])
plt.show()


    
