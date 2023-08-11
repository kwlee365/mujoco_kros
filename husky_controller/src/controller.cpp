#include "husky_controller/husky_controller.h"

using namespace Eigen;

// std::ofstream fout("/home/kwan/data/FT_circle.txt");
// std::ofstream fout("/home/kwan/data/FT_square.txt");
std::ofstream fout("/home/kwan/data/FT_eight.txt");
// std::ofstream fout("/home/kwan/data/circle_cylinder.txt");
// std::ofstream fout("/home/kwan/data/circle_sphere.txt");

HuskyController::HuskyController(ros::NodeHandle &nh, DataContainer &dc, int control_mode) : dc_(dc)
{
    hz_ = dc_.hz_;

    if (control_mode == 0)
        dc_.sim_mode_ = "position";
    else if (control_mode == 1)
        dc_.sim_mode_ = "torque";

    // RBDL
    bool floating_base = false;
    bool verbose = false;

    std::string urdf_name = ros::package::getPath("husky_description") + "/robots/panda_arm.urdf";
    std::cout << "Model name: " << urdf_name << std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &robot_, floating_base, verbose);
    // https://rbdl.github.io/d6/db6/namespace_rigid_body_dynamics_1_1_addons.html#af01cd376f05076b855ad55a625d95065

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Keyboard
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
                // Robot state
                q_.setZero();
                q_dot_.setZero();
                effort_.setZero();

                j_temp_.resize(6, PANDA_DOF);
                j_temp_.setZero();
                j_.resize(6, PANDA_DOF);
                j_.setZero();
                j_inverse_.resize(PANDA_DOF, 6);
                j_.setZero();
                j_v_.resize(3, PANDA_DOF);
                j_v_.setZero();
                j_w_.resize(3, PANDA_DOF);
                j_w_.setZero();

                x_.linear().setZero();
                x_.translation().setZero();
                x_dot_.resize(6);
                x_dot_.setZero();
                x_dot_desired_.resize(6);
                x_dot_desired_.setZero();

                // Control
                q_ddot_desired_.resize(PANDA_DOF);
                q_ddot_desired_.setZero();
                q_dot_desired_.resize(PANDA_DOF);
                q_dot_desired_.setZero();
                q_desired_.resize(PANDA_DOF);
                q_desired_.setZero();
                torque_desired_.resize(PANDA_DOF);
                torque_desired_.setZero();
                q_target_.resize(PANDA_DOF);
                q_target_.setZero();

                kp.resize(PANDA_DOF, PANDA_DOF);
                kp.setZero();
                kv.resize(PANDA_DOF, PANDA_DOF);
                kv.setZero();
                kp_task_.resize(6, 6);
                kp_task_.setZero();
                kv_task_.resize(6, 6);
                kv_task_.setZero();

                // Control gain
                for (int i = 0; i < PANDA_DOF; i++)
                {
                    kp(i, i) = 400;
                    kv(i, i) = 100;
                }
                for (int i = 0; i < 6; i++)
                {
                    kp_task_(i, i) = 4900;
                    kv_task_(i, i) = 140;
                }

                control_input_.resize(dc_.num_dof_);
                control_input_.setZero();
                control_input_filtered_.setZero();

                non_linear_.resize(PANDA_DOF);
                non_linear_.setZero();
                g_.resize(PANDA_DOF);
                g_.setZero();
                m_.resize(PANDA_DOF, PANDA_DOF);
                m_.setZero();
                C_.resize(PANDA_DOF, PANDA_DOF);
                C_.setZero();
                Lambda_.resize(6, 6);
                Lambda_.setZero();

                // Mobile
                input_vel.resize(2);
                input_vel.setZero();
                wheel_vel.resize(4);
                wheel_vel.setZero();
                wheel_vel_measured_.resize(4);
                wheel_vel_measured_.setZero();


                // End_effector
                ee_.setZero();
                ee_contact_ = 0.0;
                
                // sensor
                force_sensor_.setZero();
                torque_sensor_.setZero();
                velocity_sensor_.setZero();
                position_sensor_.setZero();

                init_time_ = ros::Time::now().toSec(); // 초 단위로 시간을 반환.

                is_init_ = false;
            }
            else
            {
                // std::cout << "Hey, init, are you done?" << std::endl;

                cur_time_ = ros::Time::now().toSec() - init_time_;

                m_dc_.lock();
                sim_time_ = dc_.sim_time_;
                for (int i = 0; i < PANDA_DOF; i++)
                {
                    q_(i) = dc_.q_(i + WHEEL_DOF + VIRTUAL_DOF);
                    q_dot_(i) = dc_.q_dot_(i + WHEEL_DOF + VIRTUAL_DOF);
                    effort_(i) = dc_.effort_(i + WHEEL_DOF + VIRTUAL_DOF);
                }
                ee_contact_ = dc_.contact_;
                force_sensor_ = dc_.force_;
                torque_sensor_ = dc_.torque_;
                velocity_sensor_.head(3) = dc_.lin_vel_;
                velocity_sensor_.tail(3) = dc_.ang_vel_;
                position_sensor_ = dc_.pos_;

                wheel_vel_measured_ = dc_.q_dot_.segment(VIRTUAL_DOF, WHEEL_DOF);

                m_dc_.unlock();

                updateKinematicsDynamics();

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
                q_mode_init_ = q_;
                q_dot_mode_init_ = q_dot_;
                x_mode_init_ = x_;
                q_desired_ = q_;

                is_read_ = true;

                std::cout << "Mode changed to";
                // i: 105, r: 114, m: 109, s: 115, f:102, h: 104

                switch (mode_)
                {
                case (MODE_INIT):
                    std::cout << " Init Pose" << std::endl;
                    break;
                case (MODE_HOME):
                    std::cout << " Home Pose" << std::endl;
                    break;
                case (MODE_CLIK):
                    std::cout << " CLIK control" << std::endl;
                    break;
                case (MODE_MOVE):
                    std::cout << " Move husky" << std::endl;
                    break;
                case (113):
                    std::cout << " Gripper open" << std::endl;
                    break;
                case (119):
                    std::cout << " Gripper close" << std::endl;
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

void HuskyController::updateKinematicsDynamics()
{
    static const int BODY_ID = robot_.GetBodyId("panda_link7");

    // Forward kinematics
    x_.translation().setZero();
    x_.linear().setZero();

    x_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot_, q_, BODY_ID, Vector3d(0.0, 0.0, 0.0), true);
    x_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(robot_, q_, BODY_ID, true).transpose();

    // Jacobian
    j_temp_.setZero();
    j_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(robot_, q_, BODY_ID, Vector3d(0.0, 0.0, 0.0), j_temp_, true);

    for (int i = 0; i < 2; i++)
    {
        j_.block<3, PANDA_DOF>(i * 3, 0) = j_temp_.block<3, PANDA_DOF>(3 - i * 3, 0);
    }

    j_v_ = j_.block<3, PANDA_DOF>(0, 0);
    j_w_ = j_.block<3, PANDA_DOF>(3, 0);

    x_dot_ = j_ * q_dot_;

    // Coriollis + gravity
    non_linear_.setZero();
    RigidBodyDynamics::NonlinearEffects(robot_, q_, q_dot_, non_linear_);

    // Only gravity
    RigidBodyDynamics::NonlinearEffects(robot_, q_, Vector7d::Zero(), g_);

    // Mass matrix
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q_, m_, true);

    j_inverse_.setZero();
    Lambda_.setZero();
    Lambda_ = (j_ * m_.inverse() * j_.transpose()).inverse();
    j_inverse_ = m_.inverse() * j_.transpose() * Lambda_;
}

void HuskyController::computeControlInput()
{
    if (mode_ == MODE_INIT)
    {
        q_target_ << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, 0, 0;
        input_vel << 0.0, 0.0;

        moveJointPosition(q_target_, 3.0);
        // moveJointPositionTorque(q_target_, 5.0);
        moveHuskyPositionVelocity(input_vel);
    }
    else if (mode_ == MODE_HOME)
    {
        // q_target_ << 0.0, 0.00374959, 0.0, 0.00346764, 0.0, -0.0120559, 0.0;
        input_vel << 0.0, 0.0;

        // moveJointPosition(q_target_, 10.0);
        moveHuskyPositionVelocity(input_vel);

        Eigen::Vector3d target_position;
        target_position = x_mode_init_.translation();
        target_position(2) += 0.1;

        Eigen::Matrix3d target_rotation;
        target_rotation << x_mode_init_.linear();

        CLIK(target_position, target_rotation, 5.0);
    }
    else if (mode_ == MODE_CLIK)
    {
        Eigen::Vector3d target_position;
        target_position << 0.3, 0.0, 0.8;

        Eigen::Matrix3d target_rotation;
        target_rotation << 0.707, -0.707, 0.0,
                           -0.707, -0.707, 0.0,
                           0.0, 0.0, -1.0;

        input_vel << 0.0, 0.0;

        // CLIK(target_position, target_rotation, 5.0);
        saveData();
        CLIK_traj();

        moveHuskyPositionVelocity(input_vel);
    }
    else if (mode_ == MODE_NULL)
    {
    }
    else if (mode_ == MODE_MOVE)
    {
        saveData();
        Husky_traj();
    }
    else if (mode_ == 113) // q
    {
        moveEndEffector(false);
    }
    else if (mode_ == 119) // w
    {
        moveEndEffector(true);
    }
    else if (mode_ == MODE_STOP)
    {
        torque_desired_ = g_;
        wheel_vel.setZero();
        ee_.setZero();
    }
    else
    {
        torque_desired_ = g_;
        wheel_vel.setZero();
        ee_.setZero();
    }

    dc_.control_input_.setZero();
    dc_.control_input_.segment(0, WHEEL_DOF) = wheel_vel;

    if (dc_.sim_mode_ == "position")
        dc_.control_input_.segment(WHEEL_DOF, PANDA_DOF) = q_desired_;
    else if (dc_.sim_mode_ == "torque")
        dc_.control_input_.segment(WHEEL_DOF, PANDA_DOF) = torque_desired_;

    dc_.control_input_.segment(WHEEL_DOF + PANDA_DOF, EE_DOF) = ee_;
    // Virtual DOFs are located in last 6 columns
}

void HuskyController::moveJointPositionTorque(Eigen::Vector7d target_position, double duration)
{
    Vector7d q_cubic, qd_cubic, zero_vector;

    zero_vector.setZero();
    q_cubic = DyrosMath::cubicVector<7>(cur_time_,
                                        mode_init_time_,
                                        mode_init_time_ + duration,
                                        q_mode_init_,
                                        target_position,
                                        zero_vector,
                                        zero_vector);
    torque_desired_ = m_ * (kp * (q_cubic - q_) + kv * (-q_dot_)) + g_;
}

void HuskyController::moveJointPosition(Eigen::Vector7d target_position, double duration)
{
    Vector7d q_cubic, qd_cubic, zero_vector;

    zero_vector.setZero();
    q_cubic = DyrosMath::cubicVector<7>(cur_time_,
                                        mode_init_time_,
                                        mode_init_time_ + duration,
                                        q_mode_init_,
                                        target_position,
                                        zero_vector,
                                        zero_vector);

    q_desired_ = q_cubic;
}

void HuskyController::CLIK(Eigen::Vector3d target_position, Eigen::Matrix3d target_rotation, double duration)
{
    Vector6d xd_desired, x_error;

    for (int i = 0; i < 3; i++)
    {
        x_desired_.translation()(i) = DyrosMath::cubic(cur_time_,
                                                       mode_init_time_,
                                                       mode_init_time_ + duration,
                                                       x_mode_init_.translation()(i),
                                                       target_position(i),
                                                       0.0, 0.0);

        xd_desired(i) = DyrosMath::cubicDot(cur_time_,
                                            mode_init_time_,
                                            mode_init_time_ + duration,
                                            x_mode_init_.translation()(i),
                                            target_position(i),
                                            0.0, 0.0);
    }

    x_desired_.linear() = DyrosMath::rotationCubic(cur_time_,
                                                   mode_init_time_,
                                                   mode_init_time_ + duration,
                                                   x_mode_init_.linear(),
                                                   target_rotation);

    xd_desired.segment<3>(3) = DyrosMath::rotationCubicDot(cur_time_,
                                                           mode_init_time_,
                                                           mode_init_time_ + duration,
                                                           Vector3d::Zero(), Vector3d::Zero(),
                                                           x_mode_init_.linear(),
                                                           target_rotation);

    int num = 0;

    // CLIK (Control, not Planning)
    Eigen::Matrix6d Kp_;
    Kp_.setZero();
    Kp_.diagonal() << 100, 100, 100, 150, 150, 150;
    
    j_inverse_ = j_.transpose() * (j_* j_.transpose()).inverse();
    x_error.head(3) = x_desired_.translation() - x_.translation();
    x_error.tail(3) = 0.5 * 
                        (DyrosMath::skew(x_.linear().block(0,0,3,1)) * x_desired_.linear().block(0,0,3,1) + 
                         DyrosMath::skew(x_.linear().block(0,1,3,1)) * x_desired_.linear().block(0,1,3,1) + 
                         DyrosMath::skew(x_.linear().block(0,2,3,1)) * x_desired_.linear().block(0,2,3,1));	// From Advanced Robotics 77 page.
    
    q_dot_desired_ = j_inverse_ * (xd_desired + Kp_ * x_error);

    q_desired_ = q_ + q_dot_desired_ / hz_;
}

void HuskyController::CLIK_traj()
{
    Vector6d xd_desired, x_error;
    Vector7d q_dot_desired_; q_dot_desired_.setZero();

    if (is_read_ == true)
    {
        tick_limit_ = ReadTextFilePanda(panda_x_traj_, panda_z_traj_, panda_xdot_traj_, panda_zdot_traj_);

        is_save_ = true;
        is_read_ = false;
    }

    x_desired_.translation()(0) = x_mode_init_.translation()(0);
    x_desired_.translation()(1) = x_mode_init_.translation()(1) + panda_x_traj_(traj_tick_);
    x_desired_.translation()(2) = x_mode_init_.translation()(2) + panda_z_traj_(traj_tick_);

    xd_desired(0) = 0.0;
    xd_desired(1) = panda_xdot_traj_(traj_tick_);
    xd_desired(2) = panda_zdot_traj_(traj_tick_);

    x_desired_.linear() = x_mode_init_.linear();
    xd_desired.tail(3).setZero();

    // CLIK (Control, not Planning)
    Eigen::Matrix6d Kp_;
    Kp_.setZero();
    Kp_.diagonal() << 100, 100, 100, 150, 150, 150;
    
    j_inverse_ = j_.transpose() * (j_* j_.transpose()).inverse();
    x_error.head(3) = x_desired_.translation() - x_.translation();
    x_error.tail(3) = 0.5 * 
                        (DyrosMath::skew(x_.linear().block(0,0,3,1)) * x_desired_.linear().block(0,0,3,1) + 
                         DyrosMath::skew(x_.linear().block(0,1,3,1)) * x_desired_.linear().block(0,1,3,1) + 
                         DyrosMath::skew(x_.linear().block(0,2,3,1)) * x_desired_.linear().block(0,2,3,1));	// From Advanced Robotics 77 page.
    
    q_dot_desired_ = j_inverse_ * (xd_desired + Kp_ * x_error);

    q_desired_ = q_ + q_dot_desired_ / hz_;

    traj_tick_++;
    if (traj_tick_ == tick_limit_)
    {
        traj_tick_ = 0;
        is_save_ = false;
        std::cout << "Saving mode is done!" << std::endl;
    }
}

void HuskyController::Husky_traj()
{
    Vector6d xd_desired, x_error;
    if (is_read_ == true)
    {
        tick_limit_ = ReadTextFileHusky(husky_x_traj_, husky_y_traj_, husky_xdot_traj_, husky_ydot_traj_, husky_xddot_traj_, husky_yddot_traj_);

        is_read_ = false;
        is_save_ = true;
    }

    double v, w;

    v = sqrt(husky_xdot_traj_(traj_tick_)*husky_xdot_traj_(traj_tick_) + husky_ydot_traj_(traj_tick_)*husky_ydot_traj_(traj_tick_));
    
    if(v==0)
        w = 0;
    else
        w = (husky_xdot_traj_(traj_tick_) * husky_yddot_traj_(traj_tick_) - husky_ydot_traj_(traj_tick_) * husky_xddot_traj_(traj_tick_)) / (v * v);

    v = 0.25;
    w = 0.125;
    Eigen::Vector2d Husky_vel;
    Husky_vel.setZero();
    Husky_vel << v, w;

    moveHuskyPositionVelocity(Husky_vel);
    std::cout << "v_des: " << Husky_vel.transpose() << std::endl;

    traj_tick_++;
    if (traj_tick_ == tick_limit_)
    {
        traj_tick_ = 0;
        is_save_ = false;
    }
}

unsigned int HuskyController::ReadTextFilePanda(Eigen::VectorXd &x_traj, Eigen::VectorXd &y_traj, Eigen::VectorXd &xdot_traj, Eigen::VectorXd &ydot_traj)
{
    // std::string textfile_location = "/home/kwan/catkin_ws/src/husky_controller/panda_traj/circle.txt";
    std::string textfile_location = "/home/kwan/catkin_ws/src/husky_controller/panda_traj/eight.txt";
    // std::string textfile_location = "/home/kwan/catkin_ws/src/husky_controller/panda_traj/square.txt";

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

    std::cout << traj_length << std::endl;

    double time[traj_length + 1],
        ref_position_x[traj_length + 1],
        ref_position_y[traj_length + 1],
        ref_position_xdot_[traj_length + 1],
        ref_position_ydot_[traj_length + 1];

    for (int i = 0; i < traj_length + 1; i++)
    {
        fscanf(traj_file, "%lf %lf %lf %lf %lf \n",
               &time[i],
               &ref_position_x[i],
               &ref_position_y[i],
               &ref_position_xdot_[i],
               &ref_position_ydot_[i]);
    }

    x_traj.resize(traj_length + 1);
    y_traj.resize(traj_length + 1);
    xdot_traj.resize(traj_length + 1);
    ydot_traj.resize(traj_length + 1);

    for (int i = 0; i < traj_length + 1; i++)
    {
        x_traj(i) = ref_position_x[i];
        y_traj(i) = ref_position_y[i];
        xdot_traj(i) = ref_position_xdot_[i];
        ydot_traj(i) = ref_position_ydot_[i];
    }

    fclose(traj_file);

    return (traj_length + 1);
}

unsigned int HuskyController::ReadTextFileHusky(Eigen::VectorXd &x_traj, Eigen::VectorXd &y_traj, Eigen::VectorXd &xdot_traj, Eigen::VectorXd &ydot_traj, Eigen::VectorXd &xddot_traj, Eigen::VectorXd &yddot_traj)
{
    std::string textfile_location = "/home/kwan/catkin_ws/src/husky_controller/husky_traj/circle.txt";
    // std::string textfile_location = "/home/kwan/catkin_ws/src/husky_controller/husky_traj/eight.txt";
    // std::string textfile_location = "/home/kwan/catkin_ws/src/husky_controller/husky_traj/square.txt";

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

    double time[traj_length + 1],
        ref_position_x[traj_length + 1],
        ref_position_y[traj_length + 1],
        ref_position_xdot_[traj_length + 1],
        ref_position_ydot_[traj_length + 1],
        ref_position_xddot_[traj_length + 1],
        ref_position_yddot_[traj_length + 1];

    for (int i = 0; i < traj_length + 1; i++)
    {
        fscanf(traj_file, "%lf %lf %lf %lf %lf %lf %lf \n",
               &time[i],
               &ref_position_x[i],
               &ref_position_y[i],
               &ref_position_xdot_[i],
               &ref_position_ydot_[i],
               &ref_position_xddot_[i],
               &ref_position_yddot_[i]);
    }

    x_traj.resize(traj_length + 1);
    y_traj.resize(traj_length + 1);
    xdot_traj.resize(traj_length + 1);
    ydot_traj.resize(traj_length + 1);
    xddot_traj.resize(traj_length + 1);
    yddot_traj.resize(traj_length + 1);

    for (int i = 0; i < traj_length + 1; i++)
    {
        x_traj(i) = ref_position_x[i];
        y_traj(i) = ref_position_y[i];
        xdot_traj(i) = ref_position_xdot_[i];
        ydot_traj(i) = ref_position_ydot_[i];
        xddot_traj(i) = ref_position_xddot_[i];
        yddot_traj(i) = ref_position_yddot_[i];
    }

    fclose(traj_file);

    return (traj_length + 1);
}

void HuskyController::moveHuskyPositionVelocity(Eigen::Vector2d input_velocity)
{
    // http://wiki.ros.org/diff_drive_controller //
    double V = input_velocity(0);
    double w = input_velocity(1);

    // double b = 2 * 0.28545 * 1;
    double b = 2 * 0.28545 * 1.875;
    double r = 0.1651;

    double wL = (V - w * b / 2) / r;
    double wR = (V + w * b / 2) / r;

    wheel_vel(0) = wL;
    wheel_vel(2) = wL;
    wheel_vel(1) = wR;
    wheel_vel(3) = wR;
}

void HuskyController::moveEndEffector(bool is_grip)
{
    if (is_grip == false) // q
        ee_ << 0.04, 0.04;
    else if (is_grip == true)   // grasp, w
        ee_ << 0.0, 0.0;
}

void HuskyController::printData()
{
    // std::cout << "time: " << cur_time_  << "\n" 
    //           << "force: " << force_sensor_.transpose() << std::endl;

    // std::cout << "time: " << cur_time_  << "\n" 
    //           << "v: " << sqrt(velocity_sensor_(0)*velocity_sensor_(0) + velocity_sensor_(1)*velocity_sensor_(1)) << "\n"
    //           << "w : " << velocity_sensor_(5) << "\n";
}

void HuskyController::saveData()
{
    if(is_save_ == true)
    {   
        // FT
        fout << cur_time_ << " " 
             << force_sensor_.transpose() << "\n";

        // // v,w 
        // fout << cur_time_ << " "  
        //      << position_sensor_(0) << " "
        //      << position_sensor_(1) << " "
        //      << sqrt(velocity_sensor_(0)*velocity_sensor_(0) + velocity_sensor_(1)*velocity_sensor_(1)) << " "
        //      << velocity_sensor_(5) << "\n";
    }
}

Eigen::MatrixXd HuskyController::JacobianUpdate(Eigen::Vector7d qd_)
{
    static const int BODY_ID = robot_.GetBodyId("panda_link7");

    Eigen::MatrixXd j_temp_, j_qd;
    j_temp_.setZero();
    j_qd.setZero();
    j_temp_.resize(6, 7);
    j_qd.resize(6, 7);
    RigidBodyDynamics::CalcPointJacobian6D(robot_, qd_, BODY_ID, Vector3d::Zero(), j_temp_, true);
    for (int i = 0; i < 2; i++)
    {
        j_qd.block<3, 7>(i * 3, 0) = j_temp_.block<3, 7>(3 - i * 3, 0);
    }

    return j_qd;
}

Eigen::Isometry3d HuskyController::PositionUpdate(Eigen::Vector7d qd_)
{
    static const int BODY_ID = robot_.GetBodyId("panda_link7");

    Isometry3d x_qd;
    x_qd.translation().setZero();
    x_qd.linear().setZero();

    x_qd.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot_, qd_, BODY_ID, Vector3d::Zero(), true);
    x_qd.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(robot_, qd_, BODY_ID, true).transpose();

    return x_qd;
}