
"""
Calculate the Koopman Operator
"""

import autograd.numpy as np
from autograd import grad, jacobian


class Koopman():
    def __init__(self, basis_f, num_basis, num_state):
        """Init a class for collecting data and training the Koopman
            Args:
                basis_f (Function) - the function for calculating the basis
                num_basis (int) - number of basis function
                num_state (int) - number of state
        """
        self.A = np.zeros((num_basis, num_basis))
        self.G = np.zeros((num_basis, num_basis))
        self.counter = 0

        self.basis = basis_f

        self.num_state = num_state



    def collect_data(self, state, new_state, action):
        """Collect data and update the Koopman
            Args:
                state (array) - a sample state
                new_state (array) - a sample next state
                action (array) - action that takes one state to the next state
        """
        phi_curr = self.basis(state, action)
        phi_next = self.basis(new_state, action)

        self.counter += 1
        self.A += np.outer(phi_next, phi_curr)/self.counter
        self.G += np.outer(phi_curr, phi_curr)/self.counter


    def get_full_K(self):
        """Get the Koopman
            Returns:
                K (array) - the Koopman operator
        """
        return np.dot(self.A, np.linalg.pinv(self.G))


    def get_K_h_T(self):
        """Get the approximate Koopman transpose
            Returns:
                K _h_T (array) - the approximate Koopman transpose
        """
        return self.get_full_K()[:self.num_state, :]        


