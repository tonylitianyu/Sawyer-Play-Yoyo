#include "ros/ros.h"
#include "sawyer_move/YoyoState.h"
#include "sawyer_move/RobotState.h"
#include "sawyer_move/RobotControl.h"
#include "std_msgs/Int8.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "env.hpp"
#include "kle.hpp"
#include "dist.hpp"


#define PY_SSIZE_T_CLEAN
#include <random>
#include <chrono>

using namespace std::chrono;
using namespace std;
using namespace Eigen;
using namespace env;
using namespace kle;
using namespace dist;

class Controller
{
    private:
        ros::Timer timer;
        ros::Subscriber start_sub;
        ros::Subscriber yoyo_state_sub;
        ros::Subscriber robot_state_sub;
        ros::Publisher control_pub;


        MatrixXd kht;
        float dt;
        int horizon;


        VectorXd state;

        int start_flag;

        Env env;
        KLE kle;
        Dist dist;
        double var;
        double kle_R;

    public:
        Controller(ros::NodeHandle nh, Env env, Dist dist, const int horizon, double var, double kle_R, int buffer_size) : 
        timer(nh.createTimer(ros::Duration(0.01), &Controller::main_loop, this)),
        yoyo_state_sub(nh.subscribe("yoyo_state", 1000, &Controller::yoyo_state_callback, this)),
        robot_state_sub(nh.subscribe("robot_state", 1000, &Controller::robot_state_callback, this)),
        start_sub(nh.subscribe("start", 1000, &Controller::start_callback, this)),
        control_pub(nh.advertise<sawyer_move::RobotControl>("robot_control", 1000, true)),
        state(VectorXd::Zero(3)),
        start_flag(0),
        env(env),
        dist(dist),
        kle(KLE(env, horizon, buffer_size)),
        var(var),
        kle_R(kle_R)
        {
        }

        void start_callback(const std_msgs::Int8 & data){
            start_flag = data.data;
        }

        void yoyo_state_callback(const sawyer_move::YoyoState & state_data){
            state[0] = state_data.yoyo_pos;
            state[1] = state_data.yoyo_posvel;
            //state[2] = state_data.yoyo_rot;
            //state[3] = state_data.yoyo_rotvel;
        }

        void robot_state_callback(const sawyer_move::RobotState & state_data){
            state[2] = state_data.ee_z_pos;
        }


        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){

            sawyer_move::RobotControl rc;
            VectorXd action = VectorXd::Zero(env.getNumActions());

            if (start_flag == 1){
                action = kle.getKLE(state, dist, 20, 40, var, kle_R, 10.0);
                // cout << "---" << endl;
                // cout << action(0) << endl;
                // cout << "---" << endl;
                rc.ee_z_vel = action(0);

                
                control_pub.publish(rc);
            }




        }


};



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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "klecontrol");
    ros::NodeHandle n;

    MatrixXd kht_down = load_csv<MatrixXd>("/home/tianyu/yoyo_ws/src/sawyer_move/src/kht_hybrid_down.csv");
    std::cout << "Koopman down hat transpose: " << kht_down << std::endl;

    MatrixXd kht_up = load_csv<MatrixXd>("/home/tianyu/yoyo_ws/src/sawyer_move/src/kht_hybrid_up.csv");
    std::cout << "Koopman up hat transpose: " << kht_up << std::endl;

    Env env = Env(kht_down, kht_up);

    MatrixXd means(1,1);
    means.col(0) << 0.8;
    cout << means << endl;

    MatrixXd sigmas(1,1);
    sigmas.col(0) << 0.25;
    cout << sigmas << endl;

    Dist dist = Dist(means, sigmas);

    Controller controller = Controller(n, env, dist, 10, 0.5, 1.0, 100); //0.05
    ros::spin();


    return 0;

}