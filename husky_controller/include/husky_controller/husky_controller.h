#include <mutex>    
#include <cmath>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include "mujoco_ros_msgs/JointSet.h"

#include "husky_controller/mujoco_interface.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "math_type_define.h"
#include <iostream>
#include <fstream>

#include <std_msgs/Float32MultiArray.h>

#define MODE_STOP 115
#define MODE_MOVE 109

#define WHEEL_DOF 4
#define VIRTUAL_DOF 6

class HuskyController
{
    public:
        HuskyController(ros::NodeHandle &nh, DataContainer &dc, int control_mode);
        ~HuskyController();
        void UpdateKinematics();
        void compute();
        void computeControlInput();
        void HuskyControlInput();
        unsigned int ReadTextFileHusky(Eigen::VectorXd &x_traj, Eigen::VectorXd &y_traj, Eigen::VectorXd &xdot_traj, Eigen::VectorXd &ydot_traj, Eigen::VectorXd &xddot_traj, Eigen::VectorXd &yddot_traj);
        void moveHusky(Eigen::Vector3d ref_pose, Eigen::Vector2d ref_vel);
        void DiffDriveController(Eigen::Vector2d input_velocity);
        void KanayamaController(Eigen::Vector3d ref_pose, Eigen::Vector2d ref_vel);
        void GetRelativePosition(double target_x, double target_y, double target_th,
                                 double coord_x, double coord_y, double coord_th,
                                 double &rel_x, double &rel_y, double &rel_th);
        void saveData();
        void printData();
    
    private:
        double hz_;
        double cur_time_;
        double pre_time_;
        double init_time_;

        unsigned int traj_tick_ = 0;
        unsigned int tick_limit_;
        Eigen::VectorXd husky_px, husky_py, husky_theta, husky_vx, husky_vy, husky_wz;

        Eigen::Vector3d pose_;
        Eigen::Vector3d pose_prev_;
        Eigen::Vector3d ref_pose_prev_;
        Eigen::Vector2d vel_;


        int mode_ = 0;
        double mode_init_time_ =  0.0;

        std::mutex m_dc_;   
        std::mutex m_ci_; 
        std::mutex m_ext_; 
        std::mutex m_buffer_; 
        std::mutex m_rbdl_;

        DataContainer &dc_;

        bool is_init_ = true;
        bool is_read_ = true;
        bool is_save_ = false;
        bool is_openloop_ctrl = false;
        
        double sim_time_ = 0.0;
        int traj_num_ = 0;

        Eigen::VectorXd control_input_;

        // Mobile robot
        Eigen::VectorXd input_vel;
        Eigen::VectorXd wheel_vel;
        Eigen::VectorXd wheel_vel_measured_;

        // End_effector contact force
        Eigen::Vector6d velocity_sensor_;   
        Eigen::Vector3d position_sensor_;   
        Eigen::Vector4d quat_sensor_;
};

// ##### keyboard ##### //

static struct termios initial_settings, new_settings;  
static int peek_character = -1;

void init_keyboard()
{
    tcgetattr(0, &initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}

int _kbhit()
{
    unsigned char ch;
    int nread;

    if(peek_character != -1)
        return 1;

    new_settings.c_cc[VMIN] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0, &ch, 1);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    if(nread == 1)
    {
        peek_character = ch;
        return 1;
    }

    return 0;
}

int _getch()
{
    char ch;
    
    if(peek_character != -1)
    {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }

    read(0, &ch, 1);
    return ch;
}

int _putch(int c)
{
    putchar(c);
    fflush(stdout);

    return c;
}

