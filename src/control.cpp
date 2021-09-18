#include "ros/ros.h"
#include "sawyer_move/YoyoState.h"
#include "sawyer_move/RobotState.h"
#include "sawyer_move/RobotControl.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>

using namespace Eigen;

class Controller
{
    private:
        ros::Timer timer;
        ros::Subscriber yoyo_state_sub;
        ros::Subscriber robot_state_sub;
        ros::Publisher control_pub;

        MatrixXd Q;
        MatrixXd R;
        MatrixXd kht;
        float dt;
        int horizon;

        VectorXd u;

        VectorXd state;

    public:
        Controller(ros::NodeHandle nh, const int horizon, const MatrixXd K_h_T, const MatrixXd Q, const MatrixXd R) : 
        timer(nh.createTimer(ros::Duration(0.01), &Controller::main_loop, this)),
        yoyo_state_sub(nh.subscribe("yoyo_state", 1000, &Controller::yoyo_state_callback, this)),
        robot_state_sub(nh.subscribe("robot_state", 1000, &Controller::robot_state_callback, this)),
        control_pub(nh.advertise<sawyer_move::RobotControl>("robot_control", 1000, true)),
        horizon(horizon),
        kht(K_h_T),
        Q(Q),
        R(R),
        state(VectorXd::Zero(4))
        {

            u = VectorXd::Zero(horizon);
            kht = load_csv("/home/tianyu/yoyo_ws/src/sawyer_move/src/kht.csv");
            std::cout << kht << std::endl;
        }

        void yoyo_state_callback(const sawyer_move::YoyoState & state_data){
            state[0] = state_data.yoyo_pos;
            state[1] = state_data.yoyo_posvel;
            state[2] = state_data.yoyo_rotvel;

        }

        void robot_state_callback(const sawyer_move::RobotState & state_data){
            state[3] = state_data.ee_z_pos;

        }

        //source: https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
        MatrixXd load_csv (const std::string & path) {
            std::ifstream indata;
            indata.open(path);
            std::string line;
            std::vector<double> values;
            uint rows = 0;
            
            while (std::getline(indata, line)) {
                std::stringstream lineStream(line);
                std::string cell;
                while (std::getline(lineStream, cell, ',')) {
                    values.push_back(std::stod(cell));
                }
                ++rows;
            }
            
            double* ptr = &values[0];

            Eigen::Map<Eigen::MatrixXd> res(ptr, rows, values.size()/rows);
            return res;
        }

        VectorXd basis_func(VectorXd state, float u){

            VectorXd extra_basis;
            extra_basis = VectorXd::Zero(18);
            double z = state[0];
            double zd = state[1];
            double thetad = state[2];
            double h = state[3];
            double a = u;
            // extra_basis << z,zd,thetad,h,u,u*h,u*pow(h,2),thetad*u,thetad*u*h,pow(thetad,2)*u,zd*u,zd*u*h,thetad*zd*u, pow(zd,2)*u, u*z,
            //                 u*h*z,thetad*u*z,zd*u*z;

            extra_basis(0, 0) = z;
            extra_basis(1, 0) = zd;
            extra_basis(2, 0) = thetad;
            extra_basis(3, 0) = h;
            extra_basis(4, 0) = a;
            extra_basis(5, 0) = a*h;
            extra_basis(6, 0) = a*pow(h, 2);
            extra_basis(7, 0) = a*thetad;
            extra_basis(8, 0) = a*h*thetad;
            extra_basis(9, 0) = a*pow(thetad, 2);
            extra_basis(10, 0) = a*zd;
            extra_basis(11, 0) = a*h*zd;
            extra_basis(12, 0) = a*thetad*zd;
            extra_basis(13, 0) = a*pow(zd, 2);
            extra_basis(14, 0) = a*z;
            extra_basis(15, 0) = a*h*z;
            extra_basis(16, 0) = a*thetad*z;
            extra_basis(17, 0) = a*z*zd;

            return extra_basis;
        }


        MatrixXd dbasis_dx(VectorXd state, float u){
            MatrixXd dbdx;
            dbdx = MatrixXd::Zero(18, 4);
            double z = state[0];
            double zd = state[1];
            double thetad = state[2];
            double h = state[3];
            double a = u;
            dbdx(0, 0) = 1;
            dbdx(1, 0) = 0;
            dbdx(2, 0) = 0;
            dbdx(3, 0) = 0;
            dbdx(4, 0) = 0;
            dbdx(5, 0) = 0;
            dbdx(6, 0) = 0;
            dbdx(7, 0) = 0;
            dbdx(8, 0) = 0;
            dbdx(9, 0) = 0;
            dbdx(10, 0) = 0;
            dbdx(11, 0) = 0;
            dbdx(12, 0) = 0;
            dbdx(13, 0) = 0;
            dbdx(14, 0) = a;
            dbdx(15, 0) = a*h;
            dbdx(16, 0) = a*thetad;
            dbdx(17, 0) = a*zd;
            dbdx(0, 1) = 0;
            dbdx(1, 1) = 1;
            dbdx(2, 1) = 0;
            dbdx(3, 1) = 0;
            dbdx(4, 1) = 0;
            dbdx(5, 1) = 0;
            dbdx(6, 1) = 0;
            dbdx(7, 1) = 0;
            dbdx(8, 1) = 0;
            dbdx(9, 1) = 0;
            dbdx(10, 1) = a;
            dbdx(11, 1) = a*h;
            dbdx(12, 1) = a*thetad;
            dbdx(13, 1) = 2*a*zd;
            dbdx(14, 1) = 0;
            dbdx(15, 1) = 0;
            dbdx(16, 1) = 0;
            dbdx(17, 1) = a*z;
            dbdx(0, 2) = 0;
            dbdx(1, 2) = 0;
            dbdx(2, 2) = 1;
            dbdx(3, 2) = 0;
            dbdx(4, 2) = 0;
            dbdx(5, 2) = 0;
            dbdx(6, 2) = 0;
            dbdx(7, 2) = a;
            dbdx(8, 2) = a*h;
            dbdx(9, 2) = 2*a*thetad;
            dbdx(10, 2) = 0;
            dbdx(11, 2) = 0;
            dbdx(12, 2) = a*zd;
            dbdx(13, 2) = 0;
            dbdx(14, 2) = 0;
            dbdx(15, 2) = 0;
            dbdx(16, 2) = a*z;
            dbdx(17, 2) = 0;
            dbdx(0, 3) = 0;
            dbdx(1, 3) = 0;
            dbdx(2, 3) = 0;
            dbdx(3, 3) = 1;
            dbdx(4, 3) = 0;
            dbdx(5, 3) = a;
            dbdx(6, 3) = 2*a*h;
            dbdx(7, 3) = 0;
            dbdx(8, 3) = a*thetad;
            dbdx(9, 3) = 0;
            dbdx(10, 3) = 0;
            dbdx(11, 3) = a*zd;
            dbdx(12, 3) = 0;
            dbdx(13, 3) = 0;
            dbdx(14, 3) = 0;
            dbdx(15, 3) = a*z;
            dbdx(16, 3) = 0;
            dbdx(17, 3) = 0;            


            return dbdx;
        }

        VectorXd dbasis_du(VectorXd state, float u){
            double z = state[0];
            double zd = state[1];
            double thetad = state[2];
            double h = state[3];
            double a = u;
            VectorXd dbdu;
            dbdu = VectorXd::Zero(18);

            dbdu(0, 0) = 0;
            dbdu(1, 0) = 0;
            dbdu(2, 0) = 0;
            dbdu(3, 0) = 0;
            dbdu(4, 0) = 1;
            dbdu(5, 0) = h;
            dbdu(6, 0) = pow(h, 2);
            dbdu(7, 0) = thetad;
            dbdu(8, 0) = h*thetad;
            dbdu(9, 0) = pow(thetad, 2);
            dbdu(10, 0) = zd;
            dbdu(11, 0) = h*zd;
            dbdu(12, 0) = thetad*zd;
            dbdu(13, 0) = pow(zd, 2);
            dbdu(14, 0) = z;
            dbdu(15, 0) = h*z;
            dbdu(16, 0) = thetad*z;
            dbdu(17, 0) = z*zd;

            return dbdu;
        }


        float loss_func(VectorXd state, float u){
            float loss = (state.transpose() * Q * state + u * R * u)[0];
            return loss;
        }

        VectorXd dloss_dx(VectorXd state, float u){
            VectorXd dldx = 2*Q*state;
            return dldx;
        }


        auto forward(VectorXd state, VectorXd u_traj){
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


        VectorXd backward(MatrixXd state_traj, VectorXd u_traj){
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


        VectorXd clip(VectorXd temp_u, float bound){
            VectorXd bound_arr;
            bound_arr = bound * VectorXd::Ones(temp_u.size());
            temp_u = temp_u.cwiseMin(bound_arr).cwiseMax(-bound_arr);


            return temp_u;
        }

        float get_action(VectorXd state, float init_step_size, float beta, float max_u){
            
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
            // for (int i = 0; i < 4; i++){
            //     std::cout << i << " " << state[i] << std::endl;
            // }

            sawyer_move::RobotControl rc;
            float action = get_action(state, 0.001, 5, 0.3);
            std::cout << action << std::endl;
            rc.ee_z_vel = 0.0;
            control_pub.publish(rc);

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

    

    Controller controller = Controller(n, 15, Q, Q, R);
    ros::spin();


    return 0;

}
