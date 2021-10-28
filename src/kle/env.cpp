#include "env.hpp"
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

using namespace env;


Env::Env(MatrixXd kht_down, MatrixXd kht_up) : 
kht_down(kht_down),
kht_up(kht_up)
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

void Env::getA(VectorXd state, VectorXd u, MatrixXd & currA){
    MatrixXd kht;
    if (state[1] < 0){
        kht = kht_up;
    }else{
        kht = kht_down;
    }

    MatrixXd dbdx(6,3);
    dbasis_dx(state, u, dbdx);


    MatrixXd curr_A_d;
    curr_A_d = kht * dbdx;
    MatrixXd eye;
    eye = MatrixXd::Identity(curr_A_d.rows(),curr_A_d.rows());
    currA = (curr_A_d - eye) / dt;

}

void Env::getB(VectorXd state, VectorXd u, MatrixXd & currB){
    MatrixXd kht;
    if (state[1] < 0){
        kht = kht_up;
    }else{
        kht = kht_down;
    }

    VectorXd dbdu(6);
    dbasis_du(state, u, dbdu);

    MatrixXd curr_B_d = kht * dbdu;
    currB = curr_B_d / dt;
}

VectorXd Env::step(VectorXd state, VectorXd u, MatrixXd & A, MatrixXd & B){
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
    VectorXi explore_id(1);
    explore_id << 0;
    return explore_id;
}


double Env::getdt(){
    return dt;
}




void Env::basis_func(VectorXd state, VectorXd u, VectorXd&basis_vector){
    double z = state[0];
    double zd = state[1];
    double h = state[2];
    double a = u[0];


    basis_vector(0, 0) = z;
    basis_vector(1, 0) = zd;
    basis_vector(2, 0) = h;
    basis_vector(3, 0) = sin(pow(z, 2));
    basis_vector(4, 0) = 1;
    basis_vector(5, 0) = a;

}
void Env::dbasis_dx(VectorXd state, VectorXd u, MatrixXd&dbdx){
    double z = state[0];
    double zd = state[1];
    double h = state[2];
    double a = u[0];

    dbdx(0, 0) = 1;
    dbdx(1, 0) = 0;
    dbdx(2, 0) = 0;
    dbdx(3, 0) = 2*z*cos(pow(z, 2));
    dbdx(4, 0) = 0;
    dbdx(5, 0) = 0;
    dbdx(0, 1) = 0;
    dbdx(1, 1) = 1;
    dbdx(2, 1) = 0;
    dbdx(3, 1) = 0;
    dbdx(4, 1) = 0;
    dbdx(5, 1) = 0;
    dbdx(0, 2) = 0;
    dbdx(1, 2) = 0;
    dbdx(2, 2) = 1;
    dbdx(3, 2) = 0;
    dbdx(4, 2) = 0;
    dbdx(5, 2) = 0;

}
void Env::dbasis_du(VectorXd state, VectorXd u, VectorXd&dbdu){
    double z = state[0];
    double zd = state[1];
    double h = state[2];
    double a = u[0];

    dbdu(0, 0) = 0;
    dbdu(1, 0) = 0;
    dbdu(2, 0) = 0;
    dbdu(3, 0) = 0;
    dbdu(4, 0) = 0;
    dbdu(5, 0) = 1;

}