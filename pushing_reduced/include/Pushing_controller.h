#ifndef _PUSHING_CONTROLLER_
#define _PUSHING_CONTROLLER_

//raisim pushing simulation
#include "Raisim_pushing.h"

// standard
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <filesystem>
#include <cmath>

// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_pushing_reduced.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// Image saving
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <iostream>
#include <cstdlib>
#include "stb_image_write.h"

using namespace raisim_pushing;

namespace pushing_controller{

//Acados parameters
#define NX     PUSHING_REDUCED_NX
#define NZ     PUSHING_REDUCED_NZ
#define NU     PUSHING_REDUCED_NU
#define NP     PUSHING_REDUCED_NP
#define NBX    PUSHING_REDUCED_NBX
#define NBX0   PUSHING_REDUCED_NBX0
#define NBU    PUSHING_REDUCED_NBU
#define NSBX   PUSHING_REDUCED_NSBX
#define NSBU   PUSHING_REDUCED_NSBU
#define NSH    PUSHING_REDUCED_NSH
#define NSG    PUSHING_REDUCED_NSG
#define NSPHI  PUSHING_REDUCED_NSPHI
#define NSHN   PUSHING_REDUCED_NSHN
#define NSGN   PUSHING_REDUCED_NSGN
#define NSPHIN PUSHING_REDUCED_NSPHIN
#define NSBXN  PUSHING_REDUCED_NSBXN
#define NS     PUSHING_REDUCED_NS
#define NSN    PUSHING_REDUCED_NSN
#define NG     PUSHING_REDUCED_NG
#define NBXN   PUSHING_REDUCED_NBXN
#define NGN    PUSHING_REDUCED_NGN
#define NY0    PUSHING_REDUCED_NY0
#define NY     PUSHING_REDUCED_NY
#define NYN    PUSHING_REDUCED_NYN
#define NH     PUSHING_REDUCED_NH
#define NPHI   PUSHING_REDUCED_NPHI
#define NHN    PUSHING_REDUCED_NHN
#define NPHIN  PUSHING_REDUCED_NPHIN
#define NR     PUSHING_REDUCED_NR

#define TCP_N 4
#define PASSIVE_FIL_N 11




class Pushing_controller : public Raisim_pushing{

    private:
        //ACADOS variables
        pushing_reduced_solver_capsule* acados_ocp_capsule;
        double* new_time_steps;
        int status;
        ocp_nlp_config *nlp_config;
        ocp_nlp_dims *nlp_dims;
        ocp_nlp_in *nlp_in;
        ocp_nlp_out *nlp_out;
        ocp_nlp_solver *nlp_solver;
        void *nlp_opts;
        int traj_sample;

        //MPC variables
        double current_state[NX]; //MPC current state
        double predicted_state[NX]; //MPC predicted state
        double predicted_state_dot[NX]; //MPC predicted state derivative
        double current_control[NU]; //MPC current control
        double param[NP]; //MPC parameters 
        double mpc_weights[NY]; //MPC weights
        std::vector<std::vector<float>> des_trajectory; //Object reference trajectory
        std::vector<std::vector<float>> des_trajectory_pass; //Object reference trajectory taking into account passivity

        int idxbx0[NBX0]; //index of state 
        double lbx0[NBX0]; //initial state lower bound ("initial" means "current" because at each iter MPC is initialized with current state and control)
        double ubx0[NBX0]; //initial state upper bound (equal to initial state lower bound because initial state is known)
        double x_init[NX]; //initial state
        double u0[NU]; //initial control
        double p[NP]; //parameters
        double W[NY*NY]; //transition cost weight matrix
        double W_N[NYN*NYN]; //terminal cost weight matrix
        double yref[NY]; //reference for state and control at each stage
        double yref_e[NYN]; //reference for state at terminal stage
        int NTIMINGS;
        double min_time;
        double kkt_norm_inf;
        double elapsed_time;
        double cost; //cost
        int sqp_iter; //sqp iteration

        double xtraj[NX * (PUSHING_REDUCED_N+1)]; //Predicted state along the prediction horizon
        double utraj[NU * PUSHING_REDUCED_N]; //Control along the prediction horizon

        double dt;//timestep

        //Tank variables
        double alpha_p; //activation parameter
        double T; //tank energy
        double T_bar; //energy upper bound
        double T_e; //energy lower bound
        double phi_p; //parameter to enforce upper bound
        double gamma_p; //parameter to enforce upper bound
        double z; //tank state
        double z_dot; //tank state derivative
        Eigen::Matrix2d lambda_1; //Gain matrix
        Eigen::Matrix2d D_d; //Damping


        
        //Obstacle state initialization
        bool obstacle_spawned;

        //Manipulator 2-D reference and state
        Eigen::Vector2d x_d; //2-D position setpoint in body frame
        Eigen::Vector2d x_d_dot; //2-D velocity setpoint in body frame
        Eigen::Vector2d x_d_world; //2-D position setpoint in world frame
        Eigen::Vector2d x_d_dot_world; //2-D velocity setpoint in world frame
        Eigen::Matrix2d box_skew; //skew symm matrix for box velocity
        Eigen::Vector2d x_d_p_world; //2-D passive position setpoint in world frame
        Eigen::Vector2d x_d_p_dot_world; //2-D passive velocity setpoint in world frame
        Eigen::Vector2d x_e_world; //2-D end-effector position
        Eigen::Vector2d x_e_dot_world; //2-D end-effector velocity
        Eigen::Vector2d f_c_p; //Predicted force in body frame
        Eigen::Vector2d f_c_p_world; //Predicted force in world frame


        //box variables
        Eigen::Vector3d box_pos; //Box position in world frame
        Eigen::Vector3d box_lin_vel; //Box linear velocity in world frame
        Eigen::Vector3d box_ang_vel; //Box angular velocity in world frame
        Eigen::Matrix3d box_R; //Box orientation in world frame
        double l; //Box dimension
        Eigen::Vector3d force_com_body; //Force at center of mass i body frame
        Eigen::Vector2d x_contact; //Contact point in plane
        Eigen::MatrixXd contact_jac; //Contact Jacobian


        //End effector variables
        Eigen::Vector3d ee_pos; //end effector position in world frame 
        Eigen::Vector3d ee_lin_vel; //end effector linear velocity in world frame
        Eigen::Vector3d ee_ang_vel; //end effector angular velocity in world frame
        Eigen::Vector3d ee_pos_body; //end effector position in body frame
        Eigen::Vector3d ee_des_pos; //end effector reference position in body frame
        Eigen::Vector3d ee_des_lin_vel; //end effector reference linear velocity in world frame
        Eigen::Vector3d ee_des_ang_vel; //end effector reference angular velocity in world frame
        Eigen::Quaterniond ee_orient; //end effector orientation in world frame
        Eigen::Matrix3d ee_des_orient_R; //end effector desired orientation in world frame
        Eigen::Vector3d f_direct; //end effector measured force
        Eigen::Vector3d f_direct_old; //end effector measured force previous step

        

        //MPC solver function
        void control_actions(); //MPC solver


        //Output data
        std::vector<std::vector<float>> current_state_vec; 
        std::vector<std::vector<float>> current_control_vec;
        std::vector<std::vector<float>> reference_TCP_not_passive_vec;
        std::vector<std::vector<float>> reference_TCP_passive_vec;
        std::vector<std::vector<float>> actual_TCP_vec;
        std::vector<std::vector<float>> passive_filter_state_vec;

        std::vector<std::vector<float>> box_vel_vec;
        std::vector<std::vector<float>> force_com_body_vec;
        void write_output();
        void write_csv(std::string, std::vector<std::vector<float>>);

        //Functions for video frame saving
        void saveScreenshot(const char* filename, unsigned char* data, int width, int height) {
            if (stbi_write_png(filename, width, height, 4, data, width * 4)) {
                std::cout << "Screenshot saved as " << filename << std::endl;
            } else {
                std::cerr << "Failed to save screenshot!" << std::endl;
            }
        }

        Window findWindowByTitle(Display*, Window, const std::string&);

    public:
        Pushing_controller(char*); //Constructor

        //set
        bool set_param(std::vector<std::vector<float>>);// Set MPC admittance parameter (stiffness)
        bool set_mpc_weights(std::vector<std::vector<float>>);// Set MPC weights
        bool set_des_trajectory(std::vector<std::vector<float>>);// Set Object reference trajectory

        //Control
        void control_loop(); //Control loop

        //Save image

        void save_video_frame(int , Display* , Window );


        //Destructor
        ~Pushing_controller(); //Destructor

};



}





















#endif