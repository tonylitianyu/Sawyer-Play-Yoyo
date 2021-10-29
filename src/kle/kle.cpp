#include "kle.hpp"
#include <vector>
#include <cmath>
#include <random>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <time.h>
#include <stdlib.h>
using namespace Eigen;

using namespace kle;
using namespace std;


KLE::KLE(env::Env model, int horizon, int bufferSize) : 
model(model),
horizon(horizon),
num_states(model.getNumStates()),
num_actions(model.getNumActions()),
buffer(ReplayBuffer(bufferSize)),
currAction(MatrixXd::Zero(horizon, model.getNumActions()))
{

}

double KLE::getPsi(VectorXd explore_state, VectorXd sampleState, double var)
{
    
    VectorXd curr_sample = sampleState;
    VectorXd diff_sigma = (curr_sample - explore_state).array().square()/var;

    double sum_diff = diff_sigma.sum();
    double final_psi = exp(-0.5*sum_diff);
    return final_psi;

    
}

void KLE::getDpsiDx(VectorXd explore_state, MatrixXd& sampleStates, double var, MatrixXd& dpsidx)
{
    for (auto i = 0; i < sampleStates.rows(); i++){
        VectorXd curr_nonzero_row(explore_state.size());
        curr_nonzero_row = getPsi(explore_state, sampleStates.row(i), var) * (sampleStates.row(i) - explore_state.transpose()) / var;
        VectorXd curr_zero_row = VectorXd::Zero(model.getNumStates() - explore_state.size());
        
        VectorXd curr_row(model.getNumStates());
        
        curr_row << curr_nonzero_row, curr_zero_row;
        dpsidx.row(i) = curr_row;

    }
}

MatrixXd KLE::getDbDx(VectorXd explore_state)
{
    MatrixXd dbdx(model.getNumStates(), 1);
    for (auto i = 0; i < explore_state.size(); i++){
        dbdx(i, 0) = 80.0*pow(explore_state(i)/10.0,7.0);
    }

    for (auto j = explore_state.size(); j < (model.getNumStates() - explore_state.size()); j++){
        dbdx(j,0) = 0.0;
    }
    return dbdx;

}

void KLE::getQ(MatrixXd &currTraj, MatrixXd &sampleStates, double var, VectorXd &q)
{
    for (auto i = 0; i < sampleStates.rows(); i++){
        double q_i_sum = 0.0;
        for (auto j = 0; j < currTraj.rows(); j++){
            VectorXd currSample = sampleStates.row(i);
            VectorXd currState = currTraj.row(j);
            double q_i = getPsi(currState.head(model.getExploreDim().size()), currSample, var);
            q_i_sum += q_i;
        }
        q(i) = q_i_sum/currTraj.rows();
    }
}

void KLE::getPQratio(VectorXd &norm_p, VectorXd&norm_q, VectorXd&pq)
{
    for (auto i = 0; i < norm_p.size(); i++){
        pq(i) = norm_p(i)/norm_q(i);
    }
}

void KLE::clip(VectorXd &temp_u, float bound){
    VectorXd bound_arr;
    bound_arr = bound * VectorXd::Ones(temp_u.size());
    temp_u = temp_u.cwiseMin(bound_arr).cwiseMax(-bound_arr);

}


VectorXd KLE::getKLE(VectorXd state, Dist &goal, 
                        int batchSize, int nSample,  double var, double R, double bound)
{
    
    VectorXd p(nSample, 1);
    MatrixXd sampleStates(nSample, model.getExploreDim().size());


    goal.sampleStateSpacePDF(nSample, 2.0, -0.5, model.getExploreDim().size(), p, sampleStates);
    
    VectorXd norm_p = p/p.sum();

    
    vector<fwdDerivs> derivs_arr;
    MatrixXd currTraj(horizon, model.getNumStates());
    //forward simulation
    for (auto t = 0; t < horizon; t++){
        MatrixXd A(model.getNumStates(),model.getNumStates());
        model.getA(state, currAction.row(t), A);
        
        MatrixXd B(model.getNumStates(),model.getNumActions());
        model.getB(state, currAction.row(t), B);

        
        MatrixXd dpsidx(nSample, state.size());
        getDpsiDx(state.head(model.getExploreDim().size()), sampleStates, var, dpsidx);
        MatrixXd dbdx = getDbDx(state.head(model.getExploreDim().size()));

        struct fwdDerivs derivs;
        derivs.A = A;
        derivs.B = B;
        derivs.dpsidx = dpsidx;
        derivs.dbdx = dbdx;
        derivs_arr.push_back(derivs);

        currTraj.row(t) = state;
        state = model.step(state, VectorXd::Zero(model.getNumActions()), A, B);
    }
    

    if (buffer.getSize() > batchSize){
        MatrixXd pastStates(batchSize, model.getNumStates());
        buffer.sample(batchSize, pastStates);

        MatrixXd combineStates(currTraj.rows() + pastStates.rows(), model.getNumStates());
        combineStates << currTraj,
                         pastStates;
        currTraj = combineStates;
    }


    VectorXd q(nSample, 1);
    getQ(currTraj, sampleStates, var, q);
    VectorXd norm_q = q/q.sum();
    

    VectorXd ratio_pq(norm_p.size());
    getPQratio(norm_p, norm_q, ratio_pq);
    
    MatrixXd RI = (1.0/R)*MatrixXd::Identity(model.getNumActions(), model.getNumActions());

    VectorXd rho = VectorXd::Zero(state.size());
    //rho = rho - model.dt*( (like_ratio.T @ dpsidx).T/Ns - (dfdx + dfdu @ dmudx).T @ rho - bdx)

    VectorXd action(model.getNumActions());
    for(auto t = horizon - 1; t >= 0; t--){
        fwdDerivs currDerivs = derivs_arr[t];

        rho = rho - model.getdt()*((ratio_pq.transpose() * currDerivs.dpsidx).transpose()/nSample 
                            - (currDerivs.A).transpose() * rho - currDerivs.dbdx);
        
        VectorXd currDmuStar = -RI * currDerivs.B.transpose() * rho;
        clip(currDmuStar, bound);
        currAction.row(t) = currDmuStar;

        if (t == 0){

            action = currDmuStar;
            
        }
    }
    return action;
}

void KLE::addToBuffer(VectorXd state){
    buffer.push(state);
}

ReplayBuffer::ReplayBuffer(int max_size) : 
max_size(max_size),
buffer(deque<VectorXd>())
{
    srand (time(NULL));
}


void ReplayBuffer::push(VectorXd state){
    if (buffer.size() == max_size){
        buffer.pop_front();
    }
    buffer.push_back(state);
}
void ReplayBuffer::sample(int size, MatrixXd& pastStates){
    for (auto i = 0; i < size; i++){
        int randIdx = rand() % getSize();
        pastStates.row(i) = buffer[randIdx];
    }
}

int ReplayBuffer::getSize(){
    return buffer.size();
}