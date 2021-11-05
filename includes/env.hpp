#ifndef ENV_HPP_
#define ENV_HPP_

#include <eigen3/Eigen/Core> //sudo apt install libeigen3-dev
#include <eigen3/Eigen/Dense>
using namespace Eigen;


namespace env {

    class Env{
        public:
            Env(MatrixXd kht_down, MatrixXd kht_up);
            void getA(VectorXd state, VectorXd u, MatrixXd & currA);
            void getB(VectorXd state, VectorXd u, MatrixXd & currB);

            VectorXd step(VectorXd state, VectorXd u, MatrixXd & A, MatrixXd & B);
            int getNumStates();
            int getNumActions();
            VectorXi getExploreDim();
            double getdt();

            void basis_func(VectorXd state, VectorXd u, VectorXd&basis_vector);
            void dbasis_dx(VectorXd state, VectorXd u, MatrixXd&dbdx);
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