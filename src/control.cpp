#include "ros/ros.h"
#include "sawyer_move/YoyoState.h"
#include "sawyer_move/RobotState.h"
#include "sawyer_move/RobotControl.h"
#include "std_msgs/Int8.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <deque>

using namespace Eigen;

class MovingAverage
{
    private:
        int n_past;
        std::string name;
        std::deque<float> buffer;

    public:
        MovingAverage(const int n_past, const std::string name) : 
        n_past(n_past),
        name(name)
        {
            buffer.push_back(0.0);
        }

        void add(float new_data){
            if (buffer.size() < n_past){
                buffer.push_back(new_data);
            }else{
                buffer.pop_front();
                buffer.push_back(new_data);
            }
        }

        float avg(){
            float sum = 0.0;
            for (int i = 0; i < buffer.size(); i++){
                sum += buffer[i];
            }

            return sum/buffer.size();
        }
    
        float get_last(){
            return buffer[buffer.size() - 1];
        }

};

class Controller
{
    private:
        ros::Timer timer;
        ros::Subscriber start_sub;
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

        MovingAverage past_posvel;
        MovingAverage past_rotvel;

        int start_flag;

    public:
        Controller(ros::NodeHandle nh, const int horizon, const MatrixXd K_h_T, const MatrixXd Q, const MatrixXd R) : 
        timer(nh.createTimer(ros::Duration(0.01), &Controller::main_loop, this)),
        yoyo_state_sub(nh.subscribe("yoyo_state", 1000, &Controller::yoyo_state_callback, this)),
        robot_state_sub(nh.subscribe("robot_state", 1000, &Controller::robot_state_callback, this)),
        start_sub(nh.subscribe("start", 1000, &Controller::start_callback, this)),
        control_pub(nh.advertise<sawyer_move::RobotControl>("robot_control", 1000, true)),
        horizon(horizon),
        kht(K_h_T),
        Q(Q),
        R(R),
        state(VectorXd::Zero(4)),
        past_posvel(MovingAverage(10, "yoyo_posvel")),
        past_rotvel(MovingAverage(10, "yoyo_rotvel")),
        start_flag(0)
        {

            u = VectorXd::Zero(horizon);
            kht = load_csv<MatrixXd>("/home/tianyu/yoyo_ws/src/sawyer_move/src/kht.csv");
            std::cout << kht << std::endl;
        }

        void start_callback(const std_msgs::Int8 & data){
            start_flag = data.data;
        }

        void yoyo_state_callback(const sawyer_move::YoyoState & state_data){
            auto temp_vel = state_data.yoyo_posvel;
            if (abs(temp_vel - past_posvel.get_last()) > 2.0){
                past_posvel.add(past_posvel.get_last());
            }else{
                past_posvel.add(temp_vel);
            }

            state[0] = state_data.yoyo_pos;
            state[1] = past_posvel.avg();//state_data.yoyo_posvel;


            auto temp_rot_vel = state_data.yoyo_rotvel;
            if (abs(temp_rot_vel) > 300.0){
                past_rotvel.add(past_rotvel.get_last());
            }else{
                past_rotvel.add(temp_rot_vel);
            }


            state[2] = past_rotvel.avg();

        }

        void robot_state_callback(const sawyer_move::RobotState & state_data){
            state[3] = state_data.ee_z_pos;

        }

        //source: https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
        template<typename M>
        M load_csv (const std::string & path) {
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
            
            //double* ptr = &values[0];

            //Eigen::Map<Eigen::MatrixXd> res(values.data(), rows, values.size()/rows);
            return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);;
        }

        VectorXd basis_func(VectorXd state, float u){

            VectorXd extra_basis;
            extra_basis = VectorXd::Zero(6);
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
            extra_basis(4, 0) = cos(pow(z, 2));
            extra_basis(5, 0) = a;

            return extra_basis;
        }


        MatrixXd dbasis_dx(VectorXd state, float u){
            MatrixXd dbdx;
            dbdx = MatrixXd::Zero(6, 4);
            double z = state[0];
            double zd = state[1];
            double thetad = state[2];
            double h = state[3];
            double a = u;
            dbdx(0, 0) = 1;
            dbdx(1, 0) = 0;
            dbdx(2, 0) = 0;
            dbdx(3, 0) = 0;
            dbdx(4, 0) = -2*z*sin(pow(z, 2));
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
            dbdx(0, 3) = 0;
            dbdx(1, 3) = 0;
            dbdx(2, 3) = 0;
            dbdx(3, 3) = 1;
            dbdx(4, 3) = 0;
            dbdx(5, 3) = 0;        


            return dbdx;
        }

        VectorXd dbasis_du(VectorXd state, float u){
            double z = state[0];
            double zd = state[1];
            double thetad = state[2];
            double h = state[3];
            double a = u;
            VectorXd dbdu;
            dbdu = VectorXd::Zero(6);

            dbdu(0, 0) = 0;
            dbdu(1, 0) = 0;
            dbdu(2, 0) = 0;
            dbdu(3, 0) = 0;
            dbdu(4, 0) = 0;
            dbdu(5, 0) = 1;

            return dbdu;
        }


        float loss_func(VectorXd state, VectorXd goal_state, float u){
            float loss = ((state.transpose()-goal_state.transpose()) * Q * (state - goal_state) + u * R * u)[0];
            return loss;
        }

        VectorXd dloss_dx(VectorXd state, VectorXd goal_state, float u){
            VectorXd dldx = 2*Q*(state - goal_state);
            return dldx;
        }


        auto forward(VectorXd state, VectorXd goal_state, VectorXd u_traj){
            float loss = 0.0;
            VectorXd curr_state = state;
            MatrixXd traj(state.size(), horizon);
            std::cout << "STARTING" << std::endl;
            std::cout << "u_traj" << u_traj << std::endl;
            for (int t = 0; t < horizon; t++){
                traj.col(t) = curr_state;
                loss += loss_func(curr_state, goal_state, u_traj[t]);
                std::cout << "loss" << loss << std::endl;
                std::cout << "current state" << curr_state << std::endl;
                std::cout << "u" << u_traj[t] << std::endl;

                VectorXd curr_basis = basis_func(curr_state, u_traj[t]);
                curr_state = kht * curr_basis;
            }

            std::cout << loss << std::endl;

            struct forward_res{
                MatrixXd traj;
                float loss;
            };

            return forward_res {traj, loss};
        }


        VectorXd backward(MatrixXd state_traj, VectorXd goal_state, VectorXd u_traj){
            int state_size = state_traj.rows();
            VectorXd rho;
            rho = VectorXd::Zero(state_size);

            VectorXd result_u;
            result_u = VectorXd::Zero(horizon);
            
            for (int t = horizon - 1; t >= 0; t--){
                
                VectorXd curr_dldx = dloss_dx(state_traj.col(t), goal_state, u_traj[t]);
                
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

        float get_action(VectorXd state, VectorXd goal_state, float init_step_size, float beta, float max_u){
            
            float k = init_step_size;

            auto [state_traj, loss] = forward(state, goal_state, u);
            
            VectorXd du_traj = backward(state_traj, goal_state, u);
            

            VectorXd temp_u_traj = clip(u + du_traj * k, max_u);
            auto [temp_state_traj, J2u] = forward(state, goal_state, temp_u_traj);

            float last_J2u = loss;
            while (J2u < last_J2u)
            {
                k = k * beta;
                temp_u_traj = clip(u + du_traj * k, max_u);
                auto [temp_state_traj, new_J2u] = forward(state, goal_state, temp_u_traj);
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
            float action = 0.0;
            if (start_flag == 1){
                
                VectorXd goal_state(4);
                goal_state << 0.10, 1.0, 200.0, 0.6;
                action = get_action(state, goal_state, 0.001, 5, 0.30);
            }
            //std::cout << action << std::endl;

            // std::cout << "state cost" << (state.transpose()-goal_state.transpose()) * Q * (state - goal_state) << std::endl;
            // std::cout << "input cost" << action * 1000.0 * action << std::endl;

            rc.ee_z_vel = action;
            control_pub.publish(rc);

        }
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle n;

    VectorXd Q_v(4);
    Q_v << 0.0,1.0,0.0,0.0;
    MatrixXd Q = Q_v.asDiagonal();


    VectorXd R_v(1);
    R_v << 0.1;
    MatrixXd R = R_v.asDiagonal();

    

    Controller controller = Controller(n, 10, Q, Q, R);
    ros::spin();


    return 0;

}
