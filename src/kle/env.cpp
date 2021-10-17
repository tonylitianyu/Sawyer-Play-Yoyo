#include "env.hpp"
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

using namespace env;


Env::Env()
{
    A.resize(4,4);
    A << 0.0,0.0,1.0,0.0,
         0.0,0.0,0.0,1.0,
         0.0,0.0,0.0,0.0,
         0.0,0.0,0.0,0.0;

    B.resize(4,2);
    B << 0.0,0.0,
         0.0,0.0,
         1.0,0.0,
         0.0,1.0;

}

MatrixXd Env::getA(){
    return A;
}

MatrixXd Env::getB(){
    return B;
}

VectorXd Env::step(VectorXd state, VectorXd u){
    VectorXd dstate = A*state + B*u;
    return state + dstate*dt;
}

int Env::getNumStates(){
    return num_states;
}
int Env::getNumActions(){
    return num_actions;
}

VectorXi Env::getExploreDim(){
    VectorXi explore_id(2);
    explore_id << 0, 1;
    return explore_id;
}


double Env::getdt(){
    return dt;
}
