#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


class Controller
{
    private:
        VectorXd Q_v(4);
        VectorXd R_v(1);
        float dt;

    public:
        Controller::Controller(ros::NodeHandle nh, const int horizon, const MatrixXd K_h_T, const MatrixXd Q, const MatrixXd R) : 
        timer(nh.createTimer(ros::Duration(0.01), &Controller::main_loop, this)),
        horizon(horizon),
        kht(K_h_T),
        Q(Q),
        R(R)
        {
            extra_size = 3;
            u = VectorXd::Zero(horizon);
        }

        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){

        }

}





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

    

    Controller controller = controller::Controller(50, Q, Q, R, 0.02);
    ros::spin();


    return 0;

}
