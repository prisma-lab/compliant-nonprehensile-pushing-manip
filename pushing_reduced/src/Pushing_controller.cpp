#include "Pushing_controller.h"

//Variables to change simulation. 
//_PASSIVE_FILTER_ : MPC reference passes through passive filter.

//_SPAWN_OBSTACLE_ : the osbtacle spawns at second 3 and disappears at second 5. The simulation
//stops at time 15s (because interaction happens at the beginning)

//_SAVE_VIDEO_FRAMES_: The video frames are saved in the current folder (this makes simulation very slow). Then, to generate video, run 
//ffmpeg -framerate 25 -i screenshot%d.png -c:v libx264 -r 25 -pix_fmt yuv420p output.mp4

#define _PASSIVE_FILTER_
#define _SPAWN_OBSTACLE_
//#define _SAVE_VIDEO_FRAMES_


namespace pushing_controller{

Pushing_controller::Pushing_controller(char* arg) : Raisim_pushing(arg){

    //ACADOS variables initialization
    acados_ocp_capsule = pushing_reduced_acados_create_capsule();
    new_time_steps = NULL;
    status = pushing_reduced_acados_create_with_discretization(acados_ocp_capsule, PUSHING_REDUCED_N, new_time_steps);
    if (status){
        printf("pushing_reduced_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }
    nlp_config = pushing_reduced_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = pushing_reduced_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = pushing_reduced_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = pushing_reduced_acados_get_nlp_out(acados_ocp_capsule);
    nlp_solver = pushing_reduced_acados_get_nlp_solver(acados_ocp_capsule);
    nlp_opts = pushing_reduced_acados_get_nlp_opts(acados_ocp_capsule);
    traj_sample = 0;


    //Desired trajectory vector initialization
    des_trajectory = {};
    for(int i=0;i<3; i++){des_trajectory_pass.push_back(std::vector<float>());}

    //Activation parameter initialization
    alpha_p=1;

    //Sample time
    dt = 0.001;

    //Box variables initialization
    l=0.1;
    get_box_state(box_pos, box_R, box_lin_vel, box_ang_vel);

    //End effector variables initialization
    get_ee_state(ee_pos, ee_orient, ee_lin_vel, ee_ang_vel);
    transform_world_2_box(ee_pos, ee_pos_body);
    ee_des_pos = ee_pos;
    ee_des_orient_R = ee_orient;
    ee_des_lin_vel << 0,0,0;
    ee_des_ang_vel << 0,0,0;

    //Output data initialization
    for(int i=0; i<NX; i++)
        current_state_vec.push_back(std::vector<float>());

    for(int i=0; i<NU; i++)
        current_control_vec.push_back(std::vector<float>());
    
    for(int i=0; i<TCP_N; i++){
        reference_TCP_not_passive_vec.push_back(std::vector<float>());
        reference_TCP_passive_vec.push_back(std::vector<float>());
        actual_TCP_vec.push_back(std::vector<float>());
    }
    for(int i=0; i<PASSIVE_FIL_N; i++)
        passive_filter_state_vec.push_back(std::vector<float>());

    for(int i=0; i<3; i++){
        box_vel_vec.push_back(std::vector<float>());
        force_com_body_vec.push_back(std::vector<float>());
    }

    //Force measurement initialization
    f_direct << 0,0,0;
    f_direct_old << 0,0,0;
    contact_jac.resize(2,3);
}


bool Pushing_controller::set_param(std::vector<std::vector<float>> param_vec){
    //Input data validation (on the size) and set
    if(param_vec[0].size() >= NP){
        for(int i=0; i<NP; i++){param[i]=param_vec[0][i];}
        return true;
    }
    else{return false;}
}

bool Pushing_controller::set_mpc_weights(std::vector<std::vector<float>> mpc_weights_vec){
    //Input data validation (on the size) and set
    if(mpc_weights_vec[0].size() >= NY){
        for(int i=0; i<NY; i++){mpc_weights[i]=mpc_weights_vec[0][i];}
        return true;
    }
    else{return false;}

}

bool Pushing_controller::set_des_trajectory(std::vector<std::vector<float>> des_trajectory_vec){
    //Input data validation (on the size) and set
    if(des_trajectory_vec.size() >= 6){
        des_trajectory = des_trajectory_vec;
        return true;
    }
    else{return false;}
}


void Pushing_controller::control_actions(){
//This function has been developed starting from the template generated by acados

    //set initial condition
    for(int i = 0; i<NBX0; i++){
        idxbx0[i] = i;
        lbx0[i] = current_state[i]; //lower bound on MPC state initial condition
        ubx0[i] = current_state[i]; //upper bound on MPC state initial condition
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    //initialization for state and control
    for(int i = 0; i<NX; i++){
        x_init[i] = current_state[i];  
    }
    for(int i = 0; i<NU; i++){
        u0[i] = current_control[i];         
    }

    //set parameters for all N stages 
    for(int i=0; i<NP; i++){
        p[i] = param[i];
    }


    //Parameters legend
    //p[0] = L_x 
    //p[1] = L_y 
    //p[2] = L_theta 
    //p[3] = mu
    //p[4] = k_x stiffness x
    //p[5] = k_y stiffness y
    //p[6] = d_x damping x  //actually not used
    //p[7] = d_y damping y  //actually not used
    //p[8] = l box dim
    //p[9] = r end effector offset (sphere radius)

    for (int i = 0; i <= PUSHING_REDUCED_N; i++){pushing_reduced_acados_update_params(acados_ocp_capsule, i, p, NP);}
    
    //the constraints for all N stages are already properly set

    //set transition cost for N-1 stages and terminal cost for last stage N
    for (int ii=0; ii<NY*NY; ii++){W[ii] = 0.0;}
    for (int ii=0; ii<NY; ii++){W[ii+NY*ii]=mpc_weights[ii];}
    for (int ii=0; ii<NYN*NYN; ii++){W_N[ii] = 0.0;}
    for(int ii=0; ii<NYN; ii++){W_N[ii+NYN*ii]=0.1*mpc_weights[ii];}

    //Cost legend
    //W[0] = penalty on x_b_des - x_b (object position x tracking error) 
    //W[1] = penalty on y_b_des - y_b (object position y tracking error) 
    //W[2] = penalty on theta_b_des - theta_b (object position theta tracking error) 
    //W[3] = penalty on phi (meaningless)
    //W[4] = penalty on x_d (meaningless)
    //W[5] = penalty on y_d (meaningless)
    //W[6] = penalty on control input phi_dot_p
    //W[7] = penalty on control input phi_dot_m
    //W[8] = penalty on control input f_x
    //W[9] = penalty on control input f_y
    //W[10] = penalty on control input epsilon 

    for(int i=0; i<PUSHING_REDUCED_N; i++){ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);}
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, PUSHING_REDUCED_N, "W", W_N);


    //set reference for the N-1 stages (the prediction horizon is PUSHING_REDUCED_N samples)

    //Force reference is not used in this framework
    Eigen::Vector3d force_ref_com;
    Eigen::Vector2d force_ref_contact_point;
    
    //Anticipatory action aka (not used in this framework, set to 0)
    int aka = 0;
    for(int ii=0; ii<PUSHING_REDUCED_N; ii++){
        //don't exceed the size of trajectory
        if(ii+traj_sample + aka < des_trajectory[0].size()){
            //Fill the reference just for the first three state variables (x, y, theta).
            //For the others NY-3 variables (the rest of the state and the control) just set 0
            for(int jj=0; jj<NY; jj++){
                //if(jj==2){aka = 0;}else{aka = 1000;}
                if(jj<3){yref[jj]=des_trajectory[jj][ii+traj_sample + aka];}
                else{yref[jj]=0;}
            }
            //Force reference transformation at contact point (not used in this framework)
            force_ref_com << des_trajectory[3][ii+traj_sample + aka], 
                                des_trajectory[4][ii+traj_sample + aka], 
                                des_trajectory[5][ii+traj_sample + aka];

            force_ref_contact_point = (contact_jac * contact_jac.transpose()).inverse() * contact_jac * force_ref_com;
        }
        //if the size of trajectory is exceeded, just use as reference the last sample
        else{
            for(int jj=0; jj<NY; jj++){
                if(jj<3){yref[jj]=des_trajectory[jj][des_trajectory[0].size()-1];}
                else{yref[jj]=0;}
            }
            //Force reference transformation at contact point (not used in this framework)
            force_ref_com << des_trajectory[3][des_trajectory[0].size()-1], 
                                des_trajectory[4][des_trajectory[0].size()-1], 
                                des_trajectory[5][des_trajectory[0].size()-1];

            force_ref_contact_point = (contact_jac * contact_jac.transpose()).inverse() * contact_jac * force_ref_com;
        }
        //yref[6] = force_ref_contact_point[0];
        //yref[7] = force_ref_contact_point[1];
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", yref);
    }

    //Set the terminal reference for the last stage N
    if(PUSHING_REDUCED_N+traj_sample+aka < des_trajectory[0].size()){
        for(int jj=0; jj<NYN; jj++){
            if(jj<3){yref_e[jj]=des_trajectory[jj][PUSHING_REDUCED_N+traj_sample+aka];}
            else{yref_e[jj]=0;}
        }
        //Force reference transformation at contact point (not used in this framework)
        force_ref_com << des_trajectory[3][PUSHING_REDUCED_N+traj_sample+aka], 
                            des_trajectory[4][PUSHING_REDUCED_N+traj_sample+aka], 
                            des_trajectory[5][PUSHING_REDUCED_N+traj_sample+aka];

        force_ref_contact_point = (contact_jac * contact_jac.transpose()).inverse() * contact_jac * force_ref_com;
    }
    else{
        for(int jj=0; jj<NYN; jj++){
            if(jj<3){yref_e[jj]=des_trajectory[jj][des_trajectory[0].size()-1];}
            else{yref_e[jj]=0;}
        }
        //Force reference transformation at contact point (not used in this framework)
        force_ref_com << des_trajectory[3][des_trajectory[0].size()-1], 
                            des_trajectory[4][des_trajectory[0].size()-1], 
                            des_trajectory[5][des_trajectory[0].size()-1];

        force_ref_contact_point = (contact_jac * contact_jac.transpose()).inverse() * contact_jac * force_ref_com;
    }
    //yref_e[6] = force_ref_contact_point[0];
    //yref_e[7] = force_ref_contact_point[1];
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, PUSHING_REDUCED_N, "yref", yref_e);
    
    
    // prepare evaluation
    NTIMINGS = 1;
    min_time = 1e12;
    
    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++){
        // initialize solution
        for (int i = 0; i < PUSHING_REDUCED_N; i++){
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, PUSHING_REDUCED_N, "x", x_init);
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = pushing_reduced_acados_solve(acados_ocp_capsule);
        ocp_nlp_eval_cost(nlp_solver, nlp_in, nlp_out);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    // print solution and statistics 
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    
    //printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    //if (status == ACADOS_SUCCESS){printf("pushing_reduced_acados_solve(): SUCCESS!\n");}
    //else{printf("pushing_reduced_acados_solve() failed with status %d.\n", status);}

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(nlp_config, nlp_solver, "cost_value", &cost);
    //pushing_reduced_acados_print_stats(acados_ocp_capsule);

    //printf("\nSolver info:\n");
    //printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
    //       sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);
    //std::cout << " COST: " << cost << "\n";

    for(int i=0; i<NU; i++){
        current_control[i] = utraj[0*NU + i];
    }

    for(int i=0; i<NX; i++){
        predicted_state[i] = xtraj[1*NX + i];
        predicted_state_dot[i] = (xtraj[1*NX + i] - xtraj[0*NX + i])/dt;
    }
}



Pushing_controller::~Pushing_controller(){
    status = pushing_reduced_acados_free(acados_ocp_capsule);
    if (status) {
        printf("pushing_reduced_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = pushing_reduced_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("pushing_reduced_acados_free_capsule() returned status %d. \n", status);
    }
}

void Pushing_controller::write_csv(std::string name_file, std::vector<std::vector<float>> data_vec){
    std::ofstream myfile;
    myfile.open(name_file);
    for(int i=0; i<data_vec.size(); i++){
        for(std::vector<float>::iterator it = data_vec[i].begin(); it != data_vec[i].end(); ++it) {
            if (it != data_vec[i].begin()) myfile << ',';
            myfile << *it;
        }
        myfile << '\n';
    }
    myfile.close();

}





void Pushing_controller::write_output(){

    //Create a folder pushing_output_files which contains all the output_data of the simulations
    std::string home = std::string(getenv("HOME"));;
    std::string path_to_output = home + "/raisimLib/pushing_reduced/pushing_output_files/";
    std::string data_folder = "output_data";

    try {
        // create log folder in pushing_reduced folder         
        std::filesystem::create_directory(path_to_output);
        std::filesystem::create_directory(path_to_output + data_folder);
    } catch (std::runtime_error & e) {
        std::cerr << e.what() << std::endl;
    }

    std::string name_file_des_trajectory = path_to_output + data_folder + "/file_des_trajectory.csv";
    std::string name_file_current_state = path_to_output + data_folder + "/file_current_state.csv";
    std::string name_file_current_control = path_to_output + data_folder + "/file_current_control.csv";
    std::string name_file_reference_TCP_not_passive = path_to_output + data_folder + "/file_reference_TCP_not_passive.csv";
    std::string name_file_reference_TCP_passive = path_to_output + data_folder + "/file_reference_TCP_passive.csv";
    std::string name_file_actual_TCP = path_to_output + data_folder + "/file_actual_TCP.csv";
    std::string name_file_passive_filter_state = path_to_output + data_folder + "/file_passive_filter_state.csv";

    std::string name_file_box_vel = path_to_output + data_folder + "/file_box_vel.csv";
    std::string name_file_force_com_body = path_to_output + data_folder + "/file_force_com_body.csv";



    write_csv(name_file_des_trajectory, des_trajectory_pass);
    write_csv(name_file_current_state, current_state_vec);
    write_csv(name_file_current_control, current_control_vec);
    write_csv(name_file_reference_TCP_not_passive, reference_TCP_not_passive_vec);
    write_csv(name_file_reference_TCP_passive, reference_TCP_passive_vec);
    write_csv(name_file_actual_TCP, actual_TCP_vec);
    write_csv(name_file_passive_filter_state, passive_filter_state_vec);

    write_csv(name_file_box_vel, box_vel_vec);
    write_csv(name_file_force_com_body, force_com_body_vec);

}


void Pushing_controller::control_loop(){
 
    //MPC state initialization
    current_state[0] = box_pos(0); //x_b
    current_state[1] = box_pos(1); //y_b
    current_state[2]= atan2(box_R(1,0),box_R(0,0)); //theta
    current_state[3] = atan2(l/2, ee_pos_body(1)) + M_PI_2; //phi
    current_state[4]=-l/2 - 0.01; //x_d
    current_state[5]=0; //y_d
    current_state[6]=0; //f_x
    current_state[7]=0; //f_y

    //MPC predicted state initialization
    for(int i=0; i<NX; i++){
        predicted_state[i] = current_state[i];
        predicted_state_dot[i] = 0;
    }

    //MPC control initialization
    for(int i=0; i<NU; i++){current_control[i]=0;}

    //Passive filter initialization
    T_bar = 0.01;
    phi_p = 1;
    gamma_p = 1;
    z= std::sqrt(2 * T_bar);
    z_dot = 0;
    T = T_bar;
    T_e = 0.0005;
    lambda_1 = 50*Eigen::MatrixXd::Identity(2,2);

    //Manipulator reference and state initialization
    x_d << current_state[4],current_state[5];
    x_d_dot << 0,0;
    x_d_dot_world << 0,0;
    x_d_p_dot_world << 0,0;
    x_e_dot_world << 0,0;
    f_c_p << 0,0;
    f_c_p_world << 0,0;
    D_d = 50*Eigen::MatrixXd::Identity(2,2);

    
    //Obstacle state initialization
    obstacle_spawned = false;

    //video frame parameters
    int video_frame_number = 0;
    int time_sample_counter = 0;
    Display* display = XOpenDisplay(nullptr);
    if (!display) {
        std::cerr << "Failed to open X display" << std::endl;
        return;
    }
    Window root = DefaultRootWindow(display);
    std::string targetTitle = "RaiSimUnity";
    Window targetWindow = findWindowByTitle(display, root, targetTitle);




    //Control loop

    while(traj_sample < int(des_trajectory[0].size())){
        //Update desired traj passive (it's the passive version of des trajectory
        //with many samples as the simulation time samples in which the values are holded
        //when alpha_p=0. In practice it is like this reference des_trajectory_pass
        // is passed to MPC, since we pass
        // des_trajectory[i][traj_sample] but updating traj_sample only when alpha_p=1 )
        for(int i=0; i<3; i++){
            des_trajectory_pass[i].push_back(des_trajectory[i][traj_sample]);
        }

        if(traj_sample>0){
            //Update the box and manipulator state
            get_box_state(box_pos, box_R, box_lin_vel, box_ang_vel);
            get_ee_state(ee_pos, ee_orient, ee_lin_vel, ee_ang_vel);
            transform_world_2_box(ee_pos, ee_pos_body);

            //Update MPC state
            current_state[0] = box_pos(0);
            current_state[1] = box_pos(1);
            current_state[2]=atan2(box_R(1,0),box_R(0,0));
            if(current_state[2]>M_PI_2){current_state[2]=current_state[2]-2*M_PI;}
            current_state[3] = atan2(l/2, ee_pos_body(1)) + M_PI_2;
            current_state[4]=predicted_state[4];
            current_state[5]=predicted_state[5];
            current_state[6]=predicted_state[6];
            current_state[7]=predicted_state[7];

            //Get contact force measurement in body frame (not used in this control)
            double alpha_f = 0.01;
            f_direct =  box_R.transpose() * (-get_ee_ext_force());
            f_direct = alpha_f * f_direct + (1-alpha_f) * f_direct_old;
            if(f_direct[0] < 0 ){f_direct[0] = 0;}
            f_direct_old = f_direct;
            
        } 
    
        //Compute contact jacobian for force reference transformation at contact point (done in control_actions())
        //(not used in this control)
        box_lin_vel = box_R.transpose() * box_lin_vel;
        double phi_measured = atan2(l/2, ee_pos_body(1)) + M_PI_2;
        x_contact << -l/2 - param[NP-1], (l/2 + param[NP-1]) * tan(M_PI - phi_measured);
        contact_jac << 1,0,-x_contact(1), 0, 1, x_contact(0);
        

        //Run the mpc
        control_actions();

        //Non passive MPC reference in body frame
        x_d << predicted_state[4], predicted_state[5];
        x_d_dot << predicted_state_dot[4], predicted_state_dot[5];
        f_c_p << predicted_state[6], predicted_state[7];

        //Non passive reference in world frame
        get_box_state(box_pos, box_R, box_lin_vel, box_ang_vel);
        box_skew << 0,-box_ang_vel(2),box_ang_vel(2), 0;
        x_d_dot_world = box_R.block<2,2>(0,0)*x_d_dot + box_lin_vel.block<2,1>(0,0) + box_skew*box_R.block<2,2>(0,0)*x_d;
        x_d_world = box_R.block<2,2>(0,0)*x_d + box_pos.block<2,1>(0,0);

        //Predicted force in world frame
        f_c_p_world = box_R.block<2,2>(0,0)*f_c_p;

        //PASSIFICATION

        //Passive reference initialization
        if(traj_sample==0){x_d_p_world = x_d_world;}

        //End effector actual position and velocity in world frame
        x_e_world << ee_pos(0), ee_pos(1);
        x_e_dot_world<< ee_lin_vel(0), ee_lin_vel(1);

        //passive filter
        #ifdef _PASSIVE_FILTER_
            //Passive reference
            x_d_p_dot_world = alpha_p * (lambda_1*(x_d_world - x_d_p_world)+ x_d_dot_world) + (1-alpha_p)*x_e_dot_world;
            x_d_p_world = x_d_p_world + x_d_p_dot_world*dt;

            //Tank dynamics
            z_dot = phi_p/z * double((x_d_p_dot_world - x_e_dot_world).transpose() * D_d * (x_d_p_dot_world - x_e_dot_world)) - 
                gamma_p/z * double((x_d_p_dot_world - x_e_dot_world).transpose() * f_c_p_world);
            z = z + dt*z_dot;

            //Saturate tank state
            if (z < std::sqrt(2 * T_e)) {
                z = std::sqrt(2 * T_e);
                T = T_e;
            }else if (z > std::sqrt(2 * T_bar)) {
                z = std::sqrt(2 * T_bar);
                T = T_bar;
            } else {
                T = 0.5*z*z;
            }


            //Parameters update
            if(T<=T_e && ((lambda_1*(x_d_world - x_d_p_world)+ x_d_dot_world) - x_e_dot_world).transpose()*f_c_p >= 0){alpha_p=0;} else{alpha_p=1;}
            if(T < T_bar){phi_p = 1;} else{phi_p = 0;}
            if((x_d_p_dot_world - x_e_dot_world).transpose()*f_c_p < 0){gamma_p = phi_p;}else{gamma_p=1;}


        //no passive filter
        #else
            x_d_p_dot_world = x_d_dot_world;
            x_d_p_world = x_d_world;
        #endif


        //Fill manip reference with the computed one
        ee_des_pos(0) = x_d_p_world(0); ee_des_pos(1) = x_d_p_world(1);
        ee_des_lin_vel(0) = x_d_p_dot_world(0); ee_des_lin_vel(1) = x_d_p_dot_world(1); 
        
    
        //Perform manipulator motion control
        manip_motion_ctrl_1_step(ee_des_pos, ee_des_lin_vel, ee_des_orient_R, ee_des_ang_vel);

        //Spawn obstacle in time interval [3,5] seconds
        double t= get_time();
        #ifdef _SPAWN_OBSTACLE_
            if(!obstacle_spawned && t>=3 && t<5){spawn_obstacle(); obstacle_spawned = true;}
            if(obstacle_spawned && t>=5){remove_obstacle(); obstacle_spawned = false;}
        #endif


        //Save output quantities

        //MPC state and control
        for(int i=0; i<NX; i++)
            current_state_vec[i].push_back(current_state[i]);

        for(int i=0; i<NU; i++)
            current_control_vec[i].push_back(current_control[i]);
        
        //End-effector not-passive reference in world frame
        reference_TCP_not_passive_vec[0].push_back(x_d_world(0));
        reference_TCP_not_passive_vec[1].push_back(x_d_world(1));
        reference_TCP_not_passive_vec[2].push_back(x_d_dot_world(0));
        reference_TCP_not_passive_vec[3].push_back(x_d_dot_world(1));

        //End-effector state
        get_ee_state(ee_pos, ee_orient, ee_lin_vel, ee_ang_vel);
        actual_TCP_vec[0].push_back(ee_pos(0));
        actual_TCP_vec[1].push_back(ee_pos(1));
        actual_TCP_vec[2].push_back(ee_lin_vel(0));
        actual_TCP_vec[3].push_back(ee_lin_vel(1));

        //Passive filter state: passive reference in world frame, tank energy, parameters
        passive_filter_state_vec[0].push_back(x_d_p_world(0));
        passive_filter_state_vec[1].push_back(x_d_p_world(1));
        passive_filter_state_vec[2].push_back(x_d_p_dot_world(0));
        passive_filter_state_vec[3].push_back(x_d_p_dot_world(1));
        passive_filter_state_vec[4].push_back(x_d_dot(0));
        passive_filter_state_vec[5].push_back(x_d_dot(1));
        passive_filter_state_vec[6].push_back(z);    
        passive_filter_state_vec[7].push_back(T);
        passive_filter_state_vec[8].push_back(alpha_p);
        passive_filter_state_vec[9].push_back(gamma_p);
        passive_filter_state_vec[10].push_back(phi_p);

        //Save box vel and contact force at com for L identification (not used in this framework)
        force_com_body = contact_jac.transpose() * f_direct.block<2,1>(0,0);
        for(int i=0; i<3; i++){
            force_com_body_vec[i].push_back(force_com_body(i));
        }
        box_vel_vec[0].push_back(box_lin_vel(0));
        box_vel_vec[1].push_back(box_lin_vel(1));
        box_vel_vec[2].push_back(box_ang_vel(2));


        //Save video frame (40 derives from 1/dt / fps, with dt = 0.001, fps = 25)
        #ifdef _SAVE_VIDEO_FRAMES_
            if (time_sample_counter % 40 == 0) {save_video_frame(video_frame_number,display,targetWindow); video_frame_number ++;}
        #endif

        //End the simulation with obstacle at time 15s
        #ifdef _SPAWN_OBSTACLE_
            if(t>=15){break;}
        #endif

        
        //Integrate 1 step
        integrate_1_step();
        time_sample_counter++;

        //Proceed with the next sample of the trajectory if passivity is not violated
        if(alpha_p==1){traj_sample++;}

    }
    
    write_output();

    XCloseDisplay(display);

}

Window Pushing_controller::findWindowByTitle(Display* display, Window root, const std::string& title) {
    Window parent;
    Window *children;
    unsigned int nchildren;
    char* windowName = nullptr;

    // Query the list of child windows
    if (XQueryTree(display, root, &root, &parent, &children, &nchildren)) {
        for (unsigned int i = 0; i < nchildren; i++) {
            if (XFetchName(display, children[i], &windowName) > 0) {
                if (windowName && title == windowName) {
                    XFree(windowName);
                    return children[i]; // Return the window ID
                }
                XFree(windowName);
            }
            // Recursive search for child windows
            Window win = findWindowByTitle(display, children[i], title);
            if (win != 0) {
                return win;
            }
        }
        if (children) {
            XFree(children);
        }
    }
    return 0; // Return 0 if no window with the given title was found
}


void Pushing_controller::save_video_frame(int video_frame_number, Display* display, Window targetWindow){
        std::string video_frame_name = "screenshot";
        video_frame_name = video_frame_name + std::to_string(video_frame_number) + ".png";
        XWindowAttributes windowAttr;

        if (!XGetWindowAttributes(display, targetWindow, &windowAttr)) {
            std::cerr << "Failed to get window attributes" << std::endl;
            return;
        }

        int width = windowAttr.width;
        int height = windowAttr.height;

        XImage* image = XGetImage(display, targetWindow, 0, 0, width, height, AllPlanes, ZPixmap);
        if (!image) {
            std::cerr << "Failed to capture image" << std::endl;
            return;
        }

        // Allocate memory for image data in RGBA format
        unsigned char* data = new unsigned char[width * height * 4];
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                unsigned long pixel = XGetPixel(image, x, y);
                unsigned char blue = pixel & image->blue_mask;
                unsigned char green = (pixel & image->green_mask) >> 8;
                unsigned char red = (pixel & image->red_mask) >> 16;
                unsigned char alpha = 0xFF; // Fully opaque

                data[(y * width + x) * 4 + 0] = red;
                data[(y * width + x) * 4 + 1] = green;
                data[(y * width + x) * 4 + 2] = blue;
                data[(y * width + x) * 4 + 3] = alpha;
            }
        }
        saveScreenshot(video_frame_name.c_str(), data, width, height); video_frame_number++;
        delete[] data;
        XDestroyImage(image);
}

}



void read_input(std::string file_name, std::vector<std::vector<float>> & input_vec){

    std::fstream in(file_name); 

    if(!in) {
        std::cout << "Cannot open " << file_name <<  " \n";
        exit(1);
    }
  
    std::string line;
    int inn = 0;

    while (std::getline(in, line)){
        float value;
        std::stringstream ss(line);

        input_vec.push_back(std::vector<float>());

        while (ss >> value){
            input_vec[inn].push_back(value);
        }
        ++inn;
    }

}


using namespace pushing_controller;

int main(int argc, char* argv []){
    
    //Read input data from file
    std::vector<std::vector<float>> mpc_weights_vec, param_vec, des_trajectory_vec;
    read_input("/home/user/raisimLib/pushing_reduced/input_files/mpc_weights_reduced.txt", mpc_weights_vec);
    read_input("/home/user/raisimLib/pushing_reduced/input_files/param_reduced.txt", param_vec);
    read_input("/home/user/raisimLib/pushing_reduced/input_files/eight_force_2.txt", des_trajectory_vec);
    
    //Create a pushing_controller (this will launch RAISIM and configure the manipulator + box scene) 
    
    Pushing_controller controller(argv[0]);

    //Set gains and trajectory
    controller.set_param(param_vec);
    controller.set_mpc_weights(mpc_weights_vec);
    controller.set_des_trajectory(des_trajectory_vec);


    //Run control loop
    controller.control_loop();

    return 0;
}






























