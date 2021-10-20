#include "env.hpp"
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

using namespace env;


Env::Env(MatrixXd kht) : 
kht(kht)
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

    // MatrixXd curr_A_d = kht * dbasis_dx(state_traj.col(t), u_traj[t]);
    // MatrixXd eye;
    // eye = MatrixXd::Identity(curr_A_d.rows(),curr_A_d.rows());
    // MatrixXd curr_A = (curr_A_d - eye) / dt;
    return A;
}

MatrixXd Env::getB(){


    // MatrixXd curr_B_d = kht * dbasis_du(state_traj.col(t), u_traj[t]);
    // MatrixXd curr_B = curr_B_d / dt;
    return B;
}

VectorXd Env::step(VectorXd state, VectorXd u){
    //state to basis state
    //next_state = kht*basis_state

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




VectorXd Env::basis_func(VectorXd state, VectorXd u){}
MatrixXd Env::dbasis_dx(VectorXd state, VectorXd u){}
VectorXd Env::dbasis_du(VectorXd state, VectorXd u){}