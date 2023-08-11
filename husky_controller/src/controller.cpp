#include "husky_controller/husky_controller.h"

using namespace Eigen;

// std::ofstream fout("/home/kwan/data/husky/openloop_circle.txt");
// std::ofstream fout("/home/kwan/data/husky/openloop_eight.txt");
std::ofstream fout("/home/kwan/data/husky/openloop_square.txt");

HuskyController::HuskyController(ros::NodeHandle &nh, DataContainer &dc, int control_mode) : dc_(dc)
{
    hz_ = dc_.hz_;

    if (control_mode == 0)
        dc_.sim_mode_ = "position";
    else if (control_mode == 1)
        dc_.sim_mode_ = "torque";

    ros::AsyncSpinner spinner(1);
    spinner.start();

    init_keyboard();
}

HuskyController::~HuskyController()
{
}

void HuskyController::compute()
{
    ros::Rate r(hz_);

    while (ros::ok())
    {
        if (!dc_.is_first_callback)
        {
            if (is_init_)
            {
                // Mobile
                pose_prev_.setZero();
                ref_pose_prev_.setZero();

                input_vel.setZero(2);
                wheel_vel.setZero(4);
                wheel_vel_measured_.setZero(4);

                position_sensor_.setZero();
                quat_sensor_.setZero();
                velocity_sensor_.setZero();

                init_time_ = ros::Time::now().toSec(); 

                is_init_ = false;

            }
            else
            {
                cur_time_ = ros::Time::now().toSec() - init_time_;

                m_dc_.lock();
                sim_time_ = dc_.sim_time_;

                position_sensor_ = dc_.pos_;
                quat_sensor_ = dc_.ang_;
                velocity_sensor_.head(3) = dc_.lin_vel_;
                velocity_sensor_.tail(3) = dc_.ang_vel_;

                m_dc_.unlock();

                UpdateKinematics();
                computeControlInput();

                printData();
                pre_time_ = cur_time_;

            }

            if (_kbhit())
            {
                int ch = _getch();
                _putch(ch);
                mode_ = ch;

                mode_init_time_ = ros::Time::now().toSec() - init_time_;
                is_read_ = true;

                std::cout << "Mode changed to";

                switch (mode_)
                {
                case (MODE_MOVE):
                    std::cout << " Move husky" << std::endl;
                    break;
                case (MODE_STOP):
                    std::cout << " Stop Pose" << std::endl;
                    break;
                }
            }

            ros::spinOnce();
            r.sleep();
        }
    }
    close_keyboard();
}

void HuskyController::UpdateKinematics()
{
    pose_.setZero();    // px py theta
    pose_.head(2) =  position_sensor_.head(2);
    pose_(2) = DyrosMath::Quat2Euler(quat_sensor_)(2);

    vel_.setZero();     // v w
    vel_(0) = sqrt(velocity_sensor_(0)*velocity_sensor_(0) + velocity_sensor_(1)*velocity_sensor_(1));
    vel_(1) = velocity_sensor_(5);
}

void HuskyController::computeControlInput()
{
    if (mode_ == MODE_MOVE)
    {
        is_openloop_ctrl = true;   // closed loop control
        HuskyControlInput();
        saveData();
    }
    else if (mode_ == MODE_STOP)
    {
        wheel_vel.setZero();
    }
    else
    {
        wheel_vel.setZero();
    }

    dc_.control_input_.segment(0, WHEEL_DOF) = wheel_vel;
}

void HuskyController::HuskyControlInput()
{
    if (is_read_ == true)
    {
        tick_limit_ = ReadTextFileHusky(husky_px, husky_py, husky_theta, 
                                        husky_vx, husky_vy, husky_wz);

        is_read_ = false;
        is_save_ = true;
    }

    Eigen::Vector3d ref_pose; Eigen::Vector2d ref_vel;
    ref_pose.setZero(); ref_vel.setZero();
    ref_pose << husky_px(traj_tick_), husky_py(traj_tick_), husky_theta(traj_tick_);
    ref_vel  << sqrt(husky_vx(traj_tick_)*husky_vx(traj_tick_) + husky_vy(traj_tick_)*husky_vy(traj_tick_)), 
                husky_wz(traj_tick_);

    moveHusky(ref_pose, ref_vel);

    traj_tick_++;
    if (traj_tick_ == tick_limit_)
    {
        traj_tick_ = 0;
        is_save_ = false;

        std::cout << "Saving txt file is done!" << std::endl;
    }
}

unsigned int HuskyController::ReadTextFileHusky(Eigen::VectorXd &px_traj, Eigen::VectorXd &py_traj, Eigen::VectorXd &theta_traj, 
                                                Eigen::VectorXd &vx_traj, Eigen::VectorXd &vy_traj, Eigen::VectorXd &w_traj)
{
    // std::string textfile_location = "/home/kwan/mujoco_ws/src/husky_controller/husky_traj/circle.txt";
    // std::string textfile_location = "/home/kwan/mujoco_ws/src/husky_controller/husky_traj/eight.txt";
    std::string textfile_location = "/home/kwan/mujoco_ws/src/husky_controller/husky_traj/square.txt";

    FILE *traj_file = NULL;
    traj_file = fopen(textfile_location.c_str(), "r");
    int traj_length = 0;
    char tmp;

    if (traj_file == NULL)
    {
        printf("There is no txt file. Please edit code. ");
        return 0;
    }

    while (fscanf(traj_file, "%c", &tmp) != EOF)
    {
        if (tmp == '\n')
            traj_length++;
    }

    fseek(traj_file, 0L, SEEK_SET);
    traj_length -= 1;

    std::cout << "Whole trajectory ticks are : " << traj_length << std::endl; 

    double ref_position_px[traj_length + 1],
           ref_position_py[traj_length + 1],
           ref_position_theta[traj_length + 1],
           ref_position_vx[traj_length + 1],
           ref_position_vy[traj_length + 1],
           ref_position_w[traj_length + 1];

    for (int i = 0; i < traj_length + 1; i++)
    {
        fscanf(traj_file, "%lf %lf %lf %lf %lf %lf \n",
               &ref_position_px[i],
               &ref_position_py[i],
               &ref_position_theta[i],
               &ref_position_vx[i],
               &ref_position_vy[i],
               &ref_position_w[i]);
    }

    px_traj.resize(traj_length + 1);
    py_traj.resize(traj_length + 1);
    theta_traj.resize(traj_length + 1);
    vx_traj.resize(traj_length + 1);
    vy_traj.resize(traj_length + 1);
    w_traj.resize(traj_length + 1);

    for (int i = 0; i < traj_length + 1; i++)
    {
        px_traj(i) = ref_position_px[i];
        py_traj(i) = ref_position_py[i];
        theta_traj(i) = ref_position_theta[i];
        vx_traj(i) = ref_position_vx[i];
        vy_traj(i) = ref_position_vy[i];
        w_traj(i) = ref_position_w[i];
    }

    fclose(traj_file);

    return (traj_length + 1);
}

void HuskyController::moveHusky(Eigen::Vector3d ref_pose, Eigen::Vector2d ref_vel)
{
    if (is_openloop_ctrl == true)
    {
        // std::cout << "Open loop control start! " << std::endl;
        DiffDriveController(ref_vel);
    }
    else
    {
        // std::cout << "Closed loop control start! " << std::endl;
        KanayamaController(ref_pose, ref_vel);
    }
}

void HuskyController::DiffDriveController(Eigen::Vector2d input_velocity)
{
    double v = input_velocity(0);
    double w = input_velocity(1);

    double b = 2 * 0.28545 * 1.875;
    double r = 0.1651;

    double wL = (v - w * b / 2) / r;
    double wR = (v + w * b / 2) / r;

    wheel_vel(0) = wL;
    wheel_vel(2) = wL;
    wheel_vel(1) = wR;
    wheel_vel(3) = wR;
}

void HuskyController::KanayamaController(Eigen::Vector3d ref_pose, Eigen::Vector2d ref_vel)
{
    double Kx = 9.0;
    double Ky = 9.0;
    double Kth = 4.5;

    Eigen::Matrix3d Rz; Rz.setZero();
    Eigen::Vector3d error_pose; error_pose.setZero();

    Rz << cos(pose_(2)), sin(pose_(2)), 0, 
         -sin(pose_(2)), cos(pose_(2)), 0,
          0,                0,          1;

	// Error pose wrt Robot coordinate
	error_pose = Rz * (ref_pose - pose_);
	error_pose(2) = atan2(sin(error_pose(2)), cos(error_pose(2))); // angle error correction

    // Velocity getting by Kanayama Eqn
    Eigen::Vector2d opt_velocity;
    opt_velocity(0) = ref_vel(0) * cos(error_pose(2)) + (Kx * error_pose(0));
    opt_velocity(1) = ref_vel(1) + ref_vel(0) * ((Ky * error_pose(1)) + (Kth * sin(error_pose(2))));

    DiffDriveController(opt_velocity);
}

void HuskyController::GetRelativePosition(double target_x, double target_y, double target_th,
                                          double coord_x, double coord_y, double coord_th,
                                          double &rel_x, double &rel_y, double &rel_th)
{
    double rel_position_x = target_x - coord_x;
    double rel_position_y = target_y - coord_y;
    double D = sqrt(pow(rel_position_x, 2) + pow(rel_position_y, 2));
    double alpha = atan2(rel_position_y, rel_position_x);
    rel_x = D * cos(alpha - coord_th);
    rel_y = D * sin(alpha - coord_th);
    rel_th = atan2(sin(target_th - coord_th), cos(target_th - coord_th));
}

void HuskyController::saveData()
{
    if(is_save_ == true)
    {   
        // v,w 
        fout << cur_time_ << " "  
             << position_sensor_(0) << " "
             << position_sensor_(1) << "\n";
    }
}

void HuskyController::printData()
{
}
