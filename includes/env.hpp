#ifndef ENV_HPP_
#define ENV_HPP_

#include <eigen3/Eigen/Core> //sudo apt install libeigen3-dev
#include <eigen3/Eigen/Dense>
using namespace Eigen;


namespace env {

    class Env{
        public:
            Env();
            MatrixXd getA();
            MatrixXd getB();

            VectorXd step(VectorXd state, VectorXd u);
            int getNumStates();
            int getNumActions();
            VectorXi getExploreDim();
            double getdt();

        private:
            int num_states = 4;
            int num_actions = 2;
            double dt = 0.1;
            MatrixXd A;
            MatrixXd B;
            


            

    };
}

#endif