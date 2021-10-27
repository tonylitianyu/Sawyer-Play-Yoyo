import numpy as np
from koopman import Koopman
import sys
import matplotlib.pyplot as plt
from yoyo_visual import DataProcessing
np.set_printoptions(suppress=True)


state_idx = [1,2,5]

args = sys.argv #-train xxx xxx xxx -test xxx
# Koopman set up
def basis_down(state, action):
    #extra_basis = np.array([np.sin(state[0]), np.sin(state[1]),np.sin(state[2]),np.sin(state[3]),np.cos(state[0]), np.cos(state[1]),np.cos(state[2]),np.cos(state[3])])
    extra_basis = np.array([np.sin(state[0])**2, 1,action])
    psi = np.hstack((state, extra_basis))
    return psi

print(basis_down(np.zeros(len(state_idx)), 0))
num_state = len(state_idx)  #yoyo-z-pos, z-vel, rot-vel, ee-z-pos
num_basis_down = len(basis_down(np.zeros(len(state_idx)), 0))
km_down = Koopman(basis_down, num_basis_down, num_state)


def basis_up(state, action):
    #extra_basis = np.array([np.sin(state[0]), np.sin(state[1]),np.sin(state[2]),np.sin(state[3]),np.cos(state[0]), np.cos(state[1]),np.cos(state[2]),np.cos(state[3])])
    extra_basis = np.array([np.sin(state[0])**2,1,action])
    psi = np.hstack((state, extra_basis))
    return psi


print(basis_up(np.zeros(len(state_idx)), 0))
num_state = len(state_idx)  #yoyo-z-pos, z-vel, rot-vel, ee-z-pos
num_basis_up = len(basis_up(np.zeros(len(state_idx)), 0))
km_up = Koopman(basis_up, num_basis_up, num_state)
#km_down = Koopman(basis, num_basis, num_state)


print("Training data:")
for i in range(2, len(args) - 2):
    print(args[i])
    dp = DataProcessing(0, args[i])
    processed_state = dp.process()
    down_motion, up_motion = dp.split(processed_state)
    down_data_group, down_action_group = dp.create_state_pair(down_motion, state_idx)
    up_data_group, up_action_group = dp.create_state_pair(up_motion, state_idx)


    for t in range(len(down_data_group)):
        km_down.collect_data(down_data_group[t][0], down_data_group[t][1], down_action_group[t])

    for t in range(len(up_data_group)):
        km_up.collect_data(up_data_group[t][0], up_data_group[t][1], up_action_group[t])


K_down = km_down.get_full_K()
print(np.round(K_down,2))

K_up = km_up.get_full_K()
print(np.round(K_up,2))

#test
print("Test data:")
print(args[len(args) - 1])
dp = DataProcessing(0, args[len(args) - 1])
processed_state = dp.process()
#down_motion, up_motion = dp.split(processed_state)
t_step, z_pos, z_vel, rot, rot_vel, ee_pos, vel_input = dp.extract_state(processed_state)

state_list = np.hstack((z_pos.reshape(-1,1), z_vel.reshape(-1,1),ee_pos.reshape(-1,1)))
action_list = vel_input
predict_arr = []


K_h_T_down = km_down.get_K_h_T()
K_h_T_up = km_up.get_K_h_T()
state = state_list[0]
predict_state = state.copy()
freq = 200
for t in range(1,len(t_step)-1):
    if t % freq == 0:
        predict_state = state_list[t]

    if z_vel[t] < 0:
        #predict up
        predict_state = (K_h_T_up @ basis_up(predict_state, action_list[t]).reshape(-1,1)).flatten()
    else:
        #predict down
        predict_state = (K_h_T_down @ basis_down(predict_state, action_list[t]).reshape(-1,1)).flatten()

    predict_arr.append(predict_state)


predict_arr = np.array(predict_arr)
print("Loss: ")
loss = 0.0
fig, axs = plt.subplots(num_state+1)
for i in range(num_state):
    #if i == 0:
        #axs[i].set_ylim([0.0,1.0])
        #axs[i].invert_yaxis()
    axs[i].plot(range(len(predict_arr)), predict_arr[:,i])
    axs[i].plot(range(len(state_list)), state_list[:,i])
    loss += np.square(np.subtract(state_list[:,i][:-2], predict_arr[:,i])).mean()
    

action_list = np.array(action_list)
axs[num_state].plot(range(len(action_list)), action_list)
plt.show()

print(loss)
