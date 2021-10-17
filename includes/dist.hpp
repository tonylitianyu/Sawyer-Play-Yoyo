#ifndef DIST_HPP_
#define DIST_HPP_

#include "env.hpp"
#include <eigen3/Eigen/Core> //sudo apt install libeigen3-dev
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>


using namespace Eigen;
using namespace std;

namespace dist{
    class Dist{
        public:
            Dist(MatrixXd &means, MatrixXd &sigmas);
            void getPDF(MatrixXd &states, VectorXd &pdf);
            void sampleStateSpacePDF(int nSample, double high, double low, 
                                                int nExploreState, VectorXd &pdf, MatrixXd &sampleStates);

        private:
            MatrixXd means;
            MatrixXd sigmas;
            
    };
}



#endif