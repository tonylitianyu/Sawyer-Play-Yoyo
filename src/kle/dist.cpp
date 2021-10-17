#include "dist.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <thread>
#include <cmath>
using namespace Eigen;

using namespace dist;
using namespace std;


Dist::Dist(MatrixXd &means, MatrixXd &sigmas) : 
means(means),
sigmas(sigmas)
{

}


void Dist::getPDF(MatrixXd &states, VectorXd &pdf){
    pdf(states.rows());

    for (auto i = 0; i < states.rows(); i++){
        double psum = 0.0;
        for (int m = 0; m < means.cols(); m++){

            VectorXd curr_state(2);
            curr_state << states(i,0), states(i,1);
            double norm_const = 1/(2*M_PI*sqrt(sigmas(0,m)*sigmas(1,m)));
            MatrixXd bigSigInv = sigmas.col(m).asDiagonal().inverse();
            VectorXd within_exp = (curr_state - means.col(m)).transpose()*(bigSigInv)*(curr_state- means.col(m));
            psum += norm_const*exp(-0.5*within_exp(0));
        }

        pdf(i) = psum;
    }

}


void Dist::sampleStateSpacePDF(int nSample, double high, double low, int nExploreState, 
                                        VectorXd &pdf, MatrixXd &sampleStates)
{
    double HI = high; // set HI and LO according to your problem.
    double LO = low;
    double range= HI-LO;
    MatrixXd m = MatrixXd::Random(nSample,nExploreState); // 3x3 Matrix filled with random numbers between (-1,1)
    m = (m + MatrixXd::Constant(nSample,nExploreState,1.))*range/2.; // add 1 to the matrix to have values between 0 and 2; multiply with range/2
    m = (m + MatrixXd::Constant(nSample,nExploreState,LO)); //set LO as the lower bound (offset)

    for (auto i = 0; i < m.cols(); i++){
        VectorXd oneState = m.col(i);
        sampleStates.col(i) = m.col(i);
    }

    


    getPDF(sampleStates, pdf);

}