/// \file  env.cpp
/// \brief storing the Koopman model
///

#ifndef ENV_HPP_
#define ENV_HPP_

#include <eigen3/Eigen/Core> //sudo apt install libeigen3-dev
#include <eigen3/Eigen/Dense>
using namespace Eigen;


namespace env {

    class Env{
        public:
            /// \brief create the model
            /// \param kht_down koopman for downward motion
            /// \param kht_up koopman for upward motion
            Env(MatrixXd kht_down, MatrixXd kht_up);

            /// \brief get A matrix in a linear system
            /// \param state current state
            /// \param u current input
            /// \param currA current calculated A matrix
            void getA(VectorXd state, VectorXd u, MatrixXd & currA);

            /// \brief get B matrix in a linear system
            /// \param state current state
            /// \param u current input
            /// \param currA current calculated B matrix
            void getB(VectorXd state, VectorXd u, MatrixXd & currB);

            /// \brief take one step using the model
            /// \param state current state
            /// \param u current input
            /// \param A current A matrix
            /// \param B current B matrix
            /// \return next state
            VectorXd step(VectorXd state, VectorXd u, MatrixXd & A, MatrixXd & B);


            /// \brief get number of state
            /// \return number of state
            int getNumStates();

            /// \brief get number of action
            /// \return number of action
            int getNumActions();

            /// \brief get exploration dimension
            /// \return the index for the exloration dimension
            VectorXi getExploreDim();

            /// \brief get the dt
            /// \return the dt for the model
            double getdt();

            /// \brief basis function for the Koopman
            /// \param state current state
            /// \param u current input
            /// \param basis_vector store return result
            void basis_func(VectorXd state, VectorXd u, VectorXd&basis_vector);

            /// \brief calculate basis derivative respect to states
            /// \param state current state
            /// \param u current input
            /// \param dbdx store return result
            void dbasis_dx(VectorXd state, VectorXd u, MatrixXd&dbdx);

            /// \brief calculate basis derivative respect to actions
            /// \param state current state
            /// \param u current input
            /// \param dbdu store return result
            void dbasis_du(VectorXd state, VectorXd u, VectorXd&dbdu);


        private:
            int num_states = 3;
            int num_actions = 1;
            double dt = 0.01;
            MatrixXd A;
            MatrixXd B;
            MatrixXd kht_down;
            MatrixXd kht_up;
            


            

    };
}

#endif