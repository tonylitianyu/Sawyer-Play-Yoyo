#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class Controller
{
    private:
        ros::Timer timer;
        ros::Subscriber state_sub;
        ros::Publisher control_pub;

        VectorXd Q;
        VectorXd R;
        MatrixXd kht;
        float dt;
        int horizon;

        VectorXd u;


    public:
        Controller(ros::NodeHandle nh, const int horizon, const MatrixXd K_h_T, const MatrixXd Q, const MatrixXd R) : 
        timer(nh.createTimer(ros::Duration(0.01), &Controller::main_loop, this)),
        horizon(horizon),
        kht(K_h_T),
        Q(Q),
        R(R)
        {

            u = VectorXd::Zero(horizon);
        }

        float Controller::loss_func(VectorXd state, float u){
            float loss = (state.transpose() * Q * state + u * R * u)[0];
            return loss;
        }

        VectorXd Controller::dloss_dx(VectorXd state, float u){
            VectorXd dldx = 2*Q*state;
            return dldx;
        }


        auto Controller::forward(VectorXd state, VectorXd u_traj){
            float loss = 0.0;
            VectorXd curr_state = state;
            MatrixXd traj(state.size(), horizon);
            for (int t = 0; t < horizon; t++){
                traj.col(t) = curr_state;
                loss += loss_func(curr_state, u_traj[t]);

                VectorXd curr_basis = basis_func(curr_state, u_traj[t]);
                curr_state = kht * curr_basis;
            }

            struct forward_res{
                MatrixXd traj;
                float loss;
            };

            return forward_res {traj, loss};
        }


        VectorXd Controller::backward(MatrixXd state_traj, VectorXd u_traj){
            int state_size = state_traj.rows();
            VectorXd rho;
            rho = VectorXd::Zero(state_size);

            VectorXd result_u;
            result_u = VectorXd::Zero(horizon);
            
            for (int t = horizon - 1; t >= 0; t--){
                
                VectorXd curr_dldx = dloss_dx(state_traj.col(t), u_traj[t]);
                
                MatrixXd curr_A_d = kht * dbasis_dx(state_traj.col(t), u_traj[t]);

                MatrixXd curr_B_d = kht * dbasis_du(state_traj.col(t), u_traj[t]);


                MatrixXd eye;
                eye = MatrixXd::Identity(curr_A_d.rows(),curr_A_d.rows());
                MatrixXd curr_A = (curr_A_d - eye) / dt;
                MatrixXd curr_B = curr_B_d / dt;

                rho = rho - (- curr_dldx - curr_A.transpose() * rho) * dt;

            
                VectorXd du = -R.inverse() * curr_B.transpose() * rho;


                result_u[t] = du[0];

            }

            return result_u;
        }


        VectorXd Controller::clip(VectorXd temp_u, float bound){
            VectorXd bound_arr;
            bound_arr = bound * VectorXd::Ones(temp_u.size());
            temp_u = temp_u.cwiseMin(bound_arr).cwiseMax(-bound_arr);


            return temp_u;
        }

        float Controller::get_action(VectorXd state, float init_step_size, float beta, float max_u){
            
            float k = init_step_size;

            auto [state_traj, loss] = forward(state, u);
            
            VectorXd du_traj = backward(state_traj, u);
            

            VectorXd temp_u_traj = u + du_traj * k;
            auto [temp_state_traj, J2u] = forward(state, temp_u_traj);

            float last_J2u = loss;
            while (J2u < last_J2u)
            {
                k = k * beta;
                temp_u_traj = u + du_traj * k;
                auto [temp_state_traj, new_J2u] = forward(state, temp_u_traj);
                last_J2u = J2u;
                J2u = new_J2u;
            }

            k = k / beta;
            u = u + du_traj * k;


            u = clip(u, max_u);

            return u[0];    

        }


        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){

        }



};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle n;

    VectorXd Q_v(4);
    Q_v << 1.0,1.0,100.0,1.0;
    MatrixXd Q = Q_v.asDiagonal();


    VectorXd R_v(1);
    R_v << 100.0;
    MatrixXd R = R_v.asDiagonal();

    

    Controller controller = Controller(n, 50, Q, Q, R);
    ros::spin();


    return 0;

}
