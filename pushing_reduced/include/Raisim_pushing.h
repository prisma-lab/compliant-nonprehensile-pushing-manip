//This header defines the RAISIM simulation scene for non-prehensile pushing.

#ifndef _RAISIM_
#define _RAISIM_

//raisim libraries
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

namespace raisim_pushing{

//raisim simulation scene parameters
#define timestep 0.001 //raisim world timestep

//manipulator initial configuration
#define N_dof 7 //Manipulator DOF
#define q_1_init 0.85225 //joint 1 initial value
#define q_2_init 1.49536 //joint 2 initial value
#define q_3_init 1.57416 //joint 3 initial value
#define q_4_init -1.64598 //joint 4 initial value
#define q_5_init -1.49595 //joint 5 initial value
#define q_6_init 1.56783 //joint 6 initial value
#define q_7_init 2.50047 //joint 7 initial value

//manipulator control gains
#define stiffness 300
#define damping 50
#define stiffness_nullspace 1
#define damping_nullspace 0.1


//box dimentions and mass
#define box_l1 0.1 //box dimension 1
#define box_l2 0.1 //box dimension 2
#define box_l3 0.1 //box dimension 3
#define box_mass 0.5 //box mass

//box initial configuration
#define box_x_init 0 //box x position initial value
#define box_y_init 0.60 //box y position initial value
#define box_z_init 0.05 //box z position initial value 
#define box_qw_init cos(M_PI_4/2)//box Qw quaternion initial value
#define box_qx_init 0//box Qx quaternion initial value
#define box_qy_init 0//box Qy quaternion initial value
#define box_qz_init sin(M_PI_4/2)//box Qz quaternion initial value

//box material (steel) and appereance (red) are hard-coded
#define box_material "steel"
#define box_appearance "1,0,0,0.5"

//Material pair properties (brass and steel)
#define friction 0.2
#define restitution 0.95
#define res_threshold 0.001
#define static_friction 0.2
#define static_friction_threshold_velocity 0.01


class Raisim_pushing{

    private:
    
        //RAISIM simulation objects
        raisim::RaisimServer server; //server
        raisim::World world; //world
        raisim::Ground* ground; //ground
        raisim::ArticulatedSystem* lbr_iiwa; //manipulator
        raisim::Box* box; //box
        raisim::Box* obstacle; //box
        std::size_t ee_frame_index; //frame index

        std::vector<raisim::Visuals*> reference_traj_points;
        std::vector<raisim::Visuals*> box_points;
        raisim::Visuals* set_point;



        //Manipulator state
        raisim::Vec<N_dof> q; //joint positions
        raisim::Vec<N_dof> q_dot; //joint velocities
        raisim::Vec<3> ee_position; //end effector position 
        raisim::Mat<3,3> ee_orientation; //end effector orientation (expressed as rotation matrix)
        raisim::Vec<3> ee_linear_velocity; //end effector linear velocity 
        raisim::Vec<3> ee_angular_velocity; //end effector angular velocity 
        raisim::SparseJacobian J_positional; //Positional Jacobian augmented
        raisim::SparseJacobian J_rotational; //Rotational Jacobian augmented
        raisim::VecDyn gravity; //Gravity compensation
        raisim::MatDyn mass_mat; //Inertia matrix

        //Manipulator control 
        Eigen::MatrixXd K_p; //Proportional gain matrix for inverse dynamics control
        Eigen::MatrixXd K_d; //Derivative gain matrix for inverse dynamics control
        Eigen::MatrixXd K_np; //Proportional gain matrix for inverse dynamics control in Jacobian nullspace
        Eigen::MatrixXd K_nd; //Derivative gain matrix for inverse dynamics control in Jacobian nullspace
        Eigen::VectorXd y; //Auxiliar control
        Eigen::VectorXd tau_joint; //Control torques
        Eigen::VectorXd tau_null; //Null space control torques

        Eigen::VectorXd joint_nominal_config; //Joint nominal configuration reference (projected in Jacobian null space)
        Eigen::Matrix3d orientation_error_matrix; //Orientation error rotation matrix
        float error_yaw; //Orientation error Euler angles: yaw
        float error_pitch; //Orientation error Euler angles: pitch
        float error_roll; //Orientation error Euler angles: roll
        Eigen::Vector3d orientation_error_euler;  //Orientation error Euler angles vector
        Eigen::VectorXd error; //Position and orientation error
        Eigen::VectorXd error_dot; //Velocity error
        Eigen::MatrixXd J_geom; //Geometric Jacobian
        Eigen::MatrixXd J_p_inv; //Jacobian pseudoinverse
        Eigen::MatrixXd J_p_inv_w; //Jacobian weighted pseudoinverse
        Eigen::MatrixXd proj; //Null space projector

        //Box state
        Eigen::Vector3d box_position; //box position
        Eigen::Matrix3d box_orientation; //box orientation
        Eigen::Vector3d box_linear_velocity; //box linear velocity
        Eigen::Vector3d box_angular_velocity; //box angular velocity
        Eigen::Matrix3d box_skew; //box skew-symmetric operator velocity


        //State update
        void update_manipulator_state();
        void update_box_state();


    public:

        Raisim_pushing(char*); //Constructor

        void manip_motion_ctrl_1_step(Eigen::Vector3d ee_des_pos, Eigen::Vector3d ee_des_lin_vel, Eigen::Matrix3d ee_des_orient, Eigen::Vector3d ee_des_ang_vel); //Manipulator motion control (1 step: this function has to be called in an iterative loop)
        
        void set_force_on_box_1_step(Eigen::Vector3d force, Eigen::Vector3d point); //External force application on the box. The force is experssed in box (body) frame and it is applied on a point expressed in box (body) frame

        void integrate_1_step(); //Integrate one simulation step

        void get_ee_state(Eigen::Vector3d & ee_pos, Eigen::Quaterniond & ee_orient, 
                            Eigen::Vector3d & ee_lin_vel, Eigen::Vector3d & ee_ang_vel); //Get end effector state

        Eigen::Vector3d get_ee_ext_force();

        void get_box_state(Eigen::Vector3d & box_pos, Eigen::Matrix3d & box_orient, 
                            Eigen::Vector3d & box_lin_vel, Eigen::Vector3d & box_ang_vel); //Get box state

        double get_time(){return world.getWorldTime();}

        void transform_box_2_world(Eigen::Vector3d pos_in, Eigen::Vector3d vel_in, Eigen::Vector3d & pos_out, Eigen::Vector3d & vel_out); //Position and velocity transformation from box (body) frame to world frame
        void transform_world_2_box(Eigen::Vector3d pos_in, Eigen::Vector3d & pos_out); //Position transformation from world frame to box (body) frame 

        void spawn_obstacle();

        void remove_obstacle();

        void save_screenshot(){server.requestSaveScreenshot();}

        //Draw traj, box traj, setpoints
        void add_traj_point(const std::string &name, double radius, double colorR, double colorG, double colorB, double colorA, double pos_x, double pos_y, double pos_z);

        void add_box_point(const std::string &name, double radius, double colorR, double colorG, double colorB, double colorA, double pos_x, double pos_y, double pos_z);

        void add_setpoint(const std::string &name, double radius, double colorR, double colorG, double colorB, double colorA, double pos_x, double pos_y, double pos_z);

        void move_setpoint(double pos_x, double pos_y, double pos_z);


        
        ~Raisim_pushing(); //Destructor



};












}









#endif