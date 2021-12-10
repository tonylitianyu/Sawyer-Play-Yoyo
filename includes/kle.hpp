/// \file  kle.cpp
/// \brief class for calculating kle-mpc
///

#ifndef KLE_HPP_
#define KLE_HPP_

#include "env.hpp"
#include <eigen3/Eigen/Core> //sudo apt install libeigen3-dev
#include <eigen3/Eigen/Dense>
#include <deque>
#include "dist.hpp"

using namespace Eigen;
using namespace std;
using namespace dist;


namespace kle {
    /// \brief store past states
    class ReplayBuffer{
        public:
            /// \brief create object for storing past states
            /// \param max_size capacity for storing
            ReplayBuffer(int max_size);

            /// \brief push in a state
            /// \param state the state being pushed
            void push(VectorXd state);

            /// \brief sample from the past states
            /// \param size number of sample
            /// \param pastStates store return result
            void sample(int size, MatrixXd& pastStates);

            /// \brief get size of the storage
            int getSize();


        private:
            int max_size;
            deque<VectorXd> buffer;

    };

    class KLE{
        /// \brief kle-mpc
        public:
            /// \brief create kle object
            /// \param model the koopman model
            /// \param horizon prediction horizon
            /// \param bufferSize the replay buffer size
            KLE(env::Env model, int horizon, int bufferSize);

            /// \brief get calculated kle
            /// \param state the current state
            /// \param goal goal distribution
            /// \param batchSize the batch size
            /// \param nSample the number of samples
            /// \param var the variance
            /// \param R the penalty for the control
            /// \param bound bound for the control
            VectorXd getKLE(VectorXd state, Dist &goal, 
                                int batchSize, int nSample, double var, double R, double bound);

            /// \brief add state to the buffer
            /// \param state the current state
            void addToBuffer(VectorXd state);

            

        private:
            struct fwdDerivs
            {
                MatrixXd A;
                MatrixXd B;
                MatrixXd dpsidx;
                MatrixXd dbdx;
            };   

            int num_states;
            int num_actions;
            env::Env model;
            int horizon;
            ReplayBuffer buffer;
            MatrixXd currAction;

            /// \brief get the current state density
            /// \param explore_state the explored index
            /// \param sampleState the sample state
            /// \param var variance for the distribution
            /// \return psi for one state
            double getPsi(VectorXd explore_state, VectorXd sampleState, double var);

            /// \brief get the state distribution derivative
            /// \param explore_state the explored index
            /// \param sampleStates the sample states
            /// \param var variance for the distribution
            /// \param dpsidx the return result
            void getDpsiDx(VectorXd explore_state, MatrixXd &sampleStates, double var,MatrixXd& dpsidx);

            /// \brief get the barrier function derivative
            /// \param explore_state the explored index
            /// \return the dbdx
            MatrixXd getDbDx(VectorXd explore_state);

            /// \brief get the current state distribution
            /// \param currTraj current trajectory
            /// \param sampleStates sample states
            /// \param var variance for the distribution
            /// \param q the return result
            void getQ(MatrixXd &currTraj, MatrixXd &sampleStates, double var, VectorXd &q);

            /// \brief p and q ratio
            /// \param norm_p normalized p dist
            /// \param norm_q normalized q dist
            /// \param pq the return result
            void getPQratio(VectorXd &norm_p, VectorXd&norm_q, VectorXd&pq);

            /// \brief clip the calculated action
            /// \param temp_u unclip action
            /// \param bound the bound for the action
            void clip(VectorXd &temp_u, float bound);

            

            
    };

}

#endif