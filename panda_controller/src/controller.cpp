#include "panda_controller/panda_controller.h"

using namespace Eigen;

// std::ofstream fout("/home/kwan/data/panda/FT_circle.txt");
// std::ofstream fout("/home/kwan/data/panda/FT_eight.txt");
std::ofstream fout("/home/kwan/data/panda/FT_square.txt");


PandaController::PandaController(ros::NodeHandle &nh, DataContainer &dc, int control_mode) : dc_(dc)
{
    hz_ = dc_.hz_;

    if (control_mode == 0)
        dc_.sim_mode_ = "position";
    else if (control_mode == 1)
        dc_.sim_mode_ = "torque";

    // RBDL
    bool floating_base = false;
    bool verbose = false;

    std::string urdf_name = ros::package::getPath("panda_description") + "/robots/panda_arm.urdf";
    std::cout << "Model name: " << urdf_name << std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &robot_, floating_base, verbose);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    init_keyboard();
}

PandaController::~PandaController()
{
}

void PandaController::compute()
{
    ros::Rate r(hz_);

    while (ros::ok())
    {
        if (!dc_.is_first_callback)
        {
            if (is_init_)
            {
                // Robot state
                j_temp_.setZero(6, PANDA_DOF);
                j_.setZero(6, PANDA_DOF);
                j_inverse_.setZero(PANDA_DOF, 6);
                j_v_.setZero(3, PANDA_DOF);
                j_w_.setZero(3, PANDA_DOF);

                x_.linear().setZero();
                x_.translation().setZero();
                x_dot_.setZero(6);
                x_dot_desired_.setZero(6);

                // Control
                q_dot_desired_.setZero(PANDA_DOF);
                q_desired_.setZero(PANDA_DOF);
                q_target_.setZero(PANDA_DOF);

                control_input_.setZero(dc_.num_dof_);

                // End_effector
                ee_.setZero();
                ee_contact_ = 0.0;
                
                // sensor
                force_sensor_.setZero();
                torque_sensor_.setZero();

                init_time_ = ros::Time::now().toSec(); 

                is_init_ = false;
            }
            else
            {
                cur_time_ = ros::Time::now().toSec() - init_time_;

                m_dc_.lock();
                sim_time_ = dc_.sim_time_;

                for (int i = 0; i < PANDA_DOF; i++)
                {
                    q_(i) = dc_.q_(i);
                    q_dot_(i) = dc_.q_dot_(i);
                    effort_(i) = dc_.effort_(i);
                }

                force_sensor_ = dc_.force_;
                torque_sensor_ = dc_.torque_;
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
                q_mode_init_ = q_;
                q_dot_mode_init_ = q_dot_;
                x_mode_init_ = x_;
                q_desired_ = q_;

                is_read_ = true;

                std::cout << "Mode changed to";

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

void PandaController::UpdateKinematics()
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
}

void PandaController::computeControlInput()
{
    if (mode_ == MODE_INIT)
    {
        q_target_ << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, 0, 0;
        moveJointPosition(q_target_, 3.0);
    }
    else if (mode_ == MODE_HOME)
    {
        Eigen::Vector3d target_position;
        target_position = x_mode_init_.translation();
        target_position(2) += 0.1;

        Eigen::Matrix3d target_rotation;
        target_rotation << x_mode_init_.linear();

        CLIK(target_position, target_rotation, 5.0);
    }
    else if (mode_ == MODE_CLIK)
    {
        saveData();
        CLIK_traj();
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
        q_desired_.setZero();
        ee_.setZero();
    }
    else
    {
        q_desired_.setZero();
        ee_.setZero();
    }

    if (dc_.sim_mode_ == "position")
        dc_.control_input_.segment(0, PANDA_DOF) = q_desired_;
    else if (dc_.sim_mode_ == "torque")
        dc_.control_input_.segment(0, PANDA_DOF) = torque_desired_;

    dc_.control_input_.segment(PANDA_DOF, EE_DOF) = ee_;
}

void PandaController::moveJointPosition(Eigen::Vector7d target_position, double duration)
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

void PandaController::CLIK(Eigen::Vector3d target_position, Eigen::Matrix3d target_rotation, double duration)
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

void PandaController::CLIK_traj()
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

unsigned int PandaController::ReadTextFilePanda(Eigen::VectorXd &x_traj, Eigen::VectorXd &y_traj, Eigen::VectorXd &xdot_traj, Eigen::VectorXd &ydot_traj)
{
    // std::string textfile_location = "/home/kwan/mujoco_ws/src/panda_controller/panda_traj/circle.txt";
    // std::string textfile_location = "/home/kwan/mujoco_ws/src/panda_controller/panda_traj/eight.txt";
    std::string textfile_location = "/home/kwan/mujoco_ws/src/panda_controller/panda_traj/square.txt";

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

void PandaController::moveEndEffector(bool is_grip)
{
    if (is_grip == false) // q
        ee_ << 0.04, 0.04;
    else if (is_grip == true)   // grasp, w
        ee_ << 0.0, 0.0;
}

void PandaController::saveData()
{
    if(is_save_ == true)
    {   
        fout << cur_time_ << " " 
             << force_sensor_.transpose() << "\n";
    }
}

void PandaController::printData()
{
             
    std::cout <<  force_sensor_.transpose() << std::endl; 
}
