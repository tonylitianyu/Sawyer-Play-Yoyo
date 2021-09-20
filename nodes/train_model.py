import numpy as np
from koopman import Koopman
import sys
import matplotlib.pyplot as plt
from yoyo_visual import DataProcessing


args = sys.argv #-train xxx xxx xxx -test xxx


def partition1(max_range, S):
    max_range = np.asarray(max_range, dtype = int)
    a = np.indices(max_range + 1)
    b = a.sum(axis = 0) <= S
    return (a[:,b].T)

poly_basis = partition1(np.array([1,2,3,4]), 2)
print(poly_basis)




# Koopman set up
def basis(state, action):
    #extra_basis = np.array([np.sin(state[0]), np.sin(state[1]),np.sin(state[2]),np.sin(state[3]),np.cos(state[0]), np.cos(state[1]),np.cos(state[2]),np.cos(state[3])])
    extra_basis = np.array([state[1]**2, state[2]**2, state[1]**2*state[2]**2, np.cos(state[0])*action, np.cos(state[1])*action, 1.0])


    #for k in range(len(state)):
    # for p in poly_basis:
    #     curr_basis = (state[0]**p[0]) * (state[1]**p[1]) * (state[2]**p[2]) * (state[3]**p[3]) * action
    #     extra_basis = np.append(extra_basis, curr_basis)

    # for k in range(len(state)):
    #     curr_basis = np.cos(state[k])*action
    #     extra_basis = np.append(extra_basis, curr_basis)

    # for k in range(len(state)):
    #     curr_basis = np.sin(state[k])*action
    #     extra_basis = np.append(extra_basis, curr_basis) 
 
    #extra_basis = np.append(extra_basis, 1.0)

    #action_basis = np.array([action])
    psi = np.hstack((state, extra_basis))
    return psi

print(basis(np.zeros(4), 0))


num_state = 4  #yoyo-z-pos, z-vel, rot-vel, ee-z-pos
num_basis = len(basis(np.zeros(4), 0))
km = Koopman(basis, num_basis, num_state)


# Import, clean, process data
print("Training data:")
for i in range(2, len(args) - 2):
    print(args[i])
    dp = DataProcessing(0, args[i])
    t_step, z_pos, z_vel, rot, rot_vel, ee_pos, vel_input = dp.process()

    state_list = np.hstack((z_pos.reshape(-1,1), z_vel.reshape(-1,1), rot_vel.reshape(-1,1), ee_pos.reshape(-1,1)))
    action_list = vel_input


    for t in range(len(t_step)-1):
        km.collect_data(state_list[t], state_list[t+1], action_list[t])







#learn
# for t in range(len(t_step)-1):
#     km.collect_data(state_list[t], state_list[t+1], action_list[t])

K = km.get_full_K()
print(np.round(K,2))


#test
print("Test data:")
print(args[len(args) - 1])
dp = DataProcessing(0, args[len(args) - 1])
t_step, z_pos, z_vel, rot, rot_vel, ee_pos, vel_input = dp.process()

state_list = np.hstack((z_pos.reshape(-1,1), z_vel.reshape(-1,1), rot_vel.reshape(-1,1), ee_pos.reshape(-1,1)))
action_list = vel_input
predict_arr = []


K_h_T = km.get_K_h_T()
state = state_list[0]
predict_state = state.copy()
freq = 15
for t in range(1,len(t_step)-1):
    if t % freq == 0:
        predict_state = state_list[t]
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
    
np.savetxt('../src/kht.csv', np.round(km.get_K_h_T(),5), delimiter=',')
np.savetxt('../src/koopman.csv', np.round(K,5), delimiter=',')


