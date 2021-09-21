import numpy as np
from koopman import Koopman
import sys
import matplotlib.pyplot as plt
from yoyo_visual import DataProcessing



args = sys.argv #-train xxx xxx xxx -test xxx
# Koopman set up
def basis(state, action):
    #extra_basis = np.array([np.sin(state[0]), np.sin(state[1]),np.sin(state[2]),np.sin(state[3]),np.cos(state[0]), np.cos(state[1]),np.cos(state[2]),np.cos(state[3])])
    extra_basis = np.array([state[0]**2, state[1]**2, (state[0]**2)*(state[1]**2), state[3]**2,action])
    psi = np.hstack((state, extra_basis))
    return psi

print(basis(np.zeros(4), 0))


num_state = 4  #yoyo-z-pos, z-vel, rot-vel, ee-z-pos
num_basis = len(basis(np.zeros(4), 0))
km_down = Koopman(basis, num_basis, num_state)
#km_down = Koopman(basis, num_basis, num_state)


print("Training data:")
for i in range(2, len(args) - 2):
    print(args[i])
    dp = DataProcessing(0, args[i])
    processed_state = dp.process()
    up_motion = dp.split(processed_state)
    data_group, action_group = dp.create_state_pair(up_motion, [1,2,4,5])


    for t in range(len(data_group)):
        km_down.collect_data(data_group[t][0], data_group[t][1], action_group[t])


K = km_down.get_full_K()
print(np.round(K,2))

#test
print("Test data:")
print(args[len(args) - 1])
dp = DataProcessing(0, args[len(args) - 1])
processed_state = dp.process()
up_motion = dp.split(processed_state)[0:80,:]
t_step, z_pos, z_vel, rot, rot_vel, ee_pos, vel_input = dp.extract_state(processed_state)

state_list = np.hstack((z_pos.reshape(-1,1), z_vel.reshape(-1,1), rot_vel.reshape(-1,1), ee_pos.reshape(-1,1)))
action_list = vel_input
predict_arr = []


K_h_T = km_down.get_K_h_T()
state = state_list[0]
predict_state = state.copy()
freq = 10
for t in range(1,len(t_step)-1):
    if t % freq == 0:
        predict_state = state_list[t]

    if z_vel[t] < 0:
        predict_state = state_list[t]
    else:
        predict_state = (K_h_T @ basis(predict_state, action_list[t]).reshape(-1,1)).flatten()

    predict_arr.append(predict_state)


predict_arr = np.array(predict_arr)
print("Loss: ")
loss = 0.0
fig, axs = plt.subplots(num_state)
for i in range(num_state):
    if i == 0:
        #axs[i].set_ylim([0.0,1.0])
        axs[i].invert_yaxis()
    axs[i].plot(range(len(predict_arr)), predict_arr[:,i])
    axs[i].plot(range(len(state_list)), state_list[:,i])
    loss += np.square(np.subtract(state_list[:,i][:-2], predict_arr[:,i])).mean()
    
plt.show()

print(loss)