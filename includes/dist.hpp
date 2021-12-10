/// \file  dists.cpp
/// \brief Create target distribution in Eigen for KLE-MPC
///

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

            /// \brief create the initial params for the distribution, currently assume Gaussian
            /// \param means - means in Matrix form
            /// \param sigmas - sigmas in Matrix form
            Dist(MatrixXd &means, MatrixXd &sigmas);


            /// \brief get the density value at each state
            /// \param states - sampled states
            /// \param pdf - density at each state
            void getPDF(MatrixXd &states, VectorXd &pdf);

            /// \brief get sample state pdf
            /// \param nSample - number of sample
            /// \param high - largest value in state space
            /// \param low - lowest value in state space
            /// \param nExploreState - number of explore state
            /// \param pdf - the pdf later returned
            /// \param sampleStates - storing samplle states
            void sampleStateSpacePDF(int nSample, double high, double low, 
                                                int nExploreState, VectorXd &pdf, MatrixXd &sampleStates);

        private:
            MatrixXd means;
            MatrixXd sigmas;
            
    };
}



#endif