"""Visualization of the recorded yoyo and robot data
"""

import numpy as np
import matplotlib.pyplot as plt
import sys

#date = sys.argv[1]


class DataProcessing:
    def __init__(self, start_index, file_name) -> None:
        """Init an object for processing data
            Args:
                start_index (int) - the start index for visualizing data
                file_name   (str) - the file name for the data
        """
        self.line_list = []

        self.file = open("../data/"+file_name+".txt", "r")

        self.start_index = start_index


    def extract_state(self, state_list):
        """Extract state list into different variables
            Args:
                state_list (array) - the entire dataset in an 2d array
            
            Returns:
                t_step    (array) - time stamp
                z_pos     (array) - yoyo z position
                z_vel     (array) - yoyo z velocity
                rot       (array) - yoyo rotation angle
                rot_vel   (array) - yoyo rotation velocity
                ee_pos    (array) - robot endpoint z position
                vel_input (array) - control input
        """
        t_step = state_list[self.start_index:,0]
        z_pos = state_list[self.start_index:,1]
        z_vel = state_list[self.start_index:,2]
        rot = state_list[self.start_index:,3]
        rot_vel = state_list[self.start_index:,4]
        ee_pos = state_list[self.start_index:,5]
        vel_input = state_list[self.start_index:,6]

        return t_step, z_pos, z_vel, rot, rot_vel, ee_pos, vel_input

    def plot(self, state, show_time=False):
        """Plot the state
            Args:
                state     (array) - the entire dataset in an 2d array
                show_time (bool)  - the x axis is time or index
        """
        fig, axs = plt.subplots(6)
        t_step, z_pos, z_vel, rot, rot_vel, ee_pos, vel_input = self.extract_state(state)

        axs[0].plot(t_step, z_pos)
        axs[0].set_ylim([0.0,1.5])
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

    def compare(self, state1, state2):
        """Overlay two dataset to compare
            Args:
                state1 (array) - the entire first dataset in an 2d array
                state2 (array) - the entire second dataset in an 2d array
        """

        fig, axs = plt.subplots(6)
        t_step1, z_pos1, z_vel1, rot1, rot_vel1, ee_pos1, vel_input1 = self.extract_state(state1)
        t_step2, z_pos2, z_vel2, rot2, rot_vel2, ee_pos2, vel_input2 = self.extract_state(state2)

        index_step1 = range(len(z_pos1))
        index_step2 = range(len(z_pos2))
        axs[0].plot(index_step1, z_pos1)
        axs[0].plot(index_step2, z_pos2)
        axs[0].set_ylim([0.0,1.5])
        axs[0].set_title('yoyo z pos')

        axs[1].plot(index_step1, z_vel1)
        axs[1].plot(index_step2, z_vel2)
        axs[1].set_title('yoyo z vel')

        axs[2].plot(index_step1, rot1)
        axs[2].plot(index_step2, rot2)
        axs[2].set_title('yoyo rotation')

        axs[3].plot(index_step1, rot_vel1)
        axs[3].plot(index_step2, rot_vel2)
        axs[3].set_title('yoyo rot vel')

        axs[4].plot(index_step1, ee_pos1)
        axs[4].plot(index_step2, ee_pos2)
        axs[4].set_title('ee z pos')

        axs[5].plot(index_step1, vel_input1)
        axs[5].plot(index_step2, vel_input2)
        axs[5].set_title('input velocity')

        plt.show()
        

    def process(self):
        """Post processing the data (noise filtering, etc.)
            Returns:
                processed_state (array) - postprocessed data
        """
        for line in self.file:
            stripped_line = line.strip()
            curr_pos_list = stripped_line.split(', ')

            if curr_pos_list[0] != 'None':
                num_list = [float(i) for i in curr_pos_list]
                self.line_list.append(num_list)


        line_list = np.array(self.line_list)
        self.line_list = line_list
        #print(line_list)


        t_step, z_pos, z_vel, rot, rot_vel, ee_pos, vel_input = self.extract_state(line_list)

        processed_state = np.hstack((t_step.reshape(-1,1), z_pos.reshape(-1,1), z_vel.reshape(-1,1), rot.reshape(-1,1), rot_vel.reshape(-1,1), ee_pos.reshape(-1,1), vel_input.reshape(-1,1)))

        return processed_state


    def smooth_with_moving_average(self,arr, ma):
        """Noise filtering with moving average
            Args:
                arr (array) - data before filtering
                ma (int) - number of past data for average
            Returns:
                arr (array) - filtered data
        """
        arr_temp = arr.copy()
        for r in range(ma,len(arr_temp)):
            ma_sum = 0.0
            ma_count = 0

            for i in range(r - ma, r):
                ma_sum += arr_temp[i]
                ma_count += 1.0
            arr[r] = ma_sum / ma_count

        return arr


    def split(self, state_list):
        """Split data into downward motion and upward motion by checking yoyo z vel
            Args:
                state_list (array) - the entire dataset in an 2d array
            Returns:
                down_motion (array) - downward motion group
                up_motion   (array) - upward motion group
        """
        z_vel = state_list[:,2]
        up_motion = []
        down_motion = []
        for v in range(len(z_vel)):
            if z_vel[v] > 0:
                down_motion.append(state_list[v,:])
            
            else:
                up_motion.append(state_list[v,:])

        down_motion = np.array(down_motion)
        up_motion = np.array(up_motion)

        return down_motion, up_motion

    def create_state_pair(self, truncated_state, state_indexes):
        """create state pair for training Koopman
            Args:
                truncated_state (array) - either the upward motion data or downward motion data
                state_indexes   (array) - the features (states) that will be used for Koopman
            Returns:
                data_group (tuple) - the (state, next_state) pair
                action_group   (array) - action data that leads state to next_state
        """
        z_pos = truncated_state[:,1]
        data_group = []
        action_group = []
        for v in range(0, len(z_pos)-1):
            if abs(z_pos[v+1] - z_pos[v]) < 0.1:
                data_group.append([truncated_state[v,state_indexes], truncated_state[v+1,state_indexes]])
                action_group.append(truncated_state[v, -1])
            else:
                print(z_pos[v])
                #break

        return data_group, action_group






if __name__ == "__main__":
    # date1 = sys.argv[1]
    # date2 = sys.argv[2]
    # dp1 = DataProcessing(0, date1)
    # processed_state1 = dp1.process()

    # dp2 = DataProcessing(0, date2)
    # processed_state2 = dp2.process()
    # dp1.compare(processed_state1, processed_state2)



    date = sys.argv[1]


    dp = DataProcessing(0, date)
    processed_state = dp.process()

    dp.plot(processed_state)




    down_motion, up_motion = dp.split(processed_state)

    data_group, action_group = dp.create_state_pair(up_motion, [1,2,4,5])

    state_list = []

    for d in data_group:
        state_list.append(d[0])

    state_list = np.array(state_list)


    z_pos, z_vel, rot_vel, ee_pos = state_list[:,0], state_list[:,1], state_list[:,2], state_list[:,3]



    fig, axs = plt.subplots(5)


    axs[0].plot(range(len(z_pos)), z_pos)
    axs[0].set_ylim([0.0,1.5])
    axs[0].invert_yaxis()
    axs[0].set_title('yoyo z pos')

    axs[1].plot(range(len(z_vel)), z_vel)
    axs[1].set_title('yoyo z vel')

    axs[2].plot(range(len(rot_vel)), rot_vel)
    axs[2].set_title('yoyo rot vel')

    axs[3].plot(range(len(ee_pos)), ee_pos)
    axs[3].set_title('ee z pos')

    axs[4].plot(range(len(action_group)), action_group)
    axs[4].set_title('input velocity')

    plt.show()