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
    class ReplayBuffer{
        public:
            ReplayBuffer(int max_size);
            void push(VectorXd state);
            void sample(int size, MatrixXd& pastStates);
            int getSize();


        private:
            int max_size;
            deque<VectorXd> buffer;

    };

    class KLE{
        public:
            KLE(env::Env model, int horizon, int bufferSize);
            VectorXd getKLE(VectorXd state, Dist &goal, 
                                int batchSize, int nSample, double var, double R, double bound);

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

            double getPsi(VectorXd explore_state, VectorXd sampleState, double var);
            void getDpsiDx(VectorXd explore_state, MatrixXd &sampleStates, double var,MatrixXd& dpsidx);
            MatrixXd getDbDx(VectorXd explore_state);
            void getQ(MatrixXd &currTraj, MatrixXd &sampleStates, double var, VectorXd &q);
            void getPQratio(VectorXd &norm_p, VectorXd&norm_q, VectorXd&pq);
            void clip(VectorXd &temp_u, float bound);

            

            
    };

}

#endif