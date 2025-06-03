#include "Raisim_pushing.h"

namespace raisim_pushing{

Raisim_pushing::Raisim_pushing(char* arg): world(), server(&world){ //create world and server at the construction

    //Resize vectors
    J_positional.resize(N_dof);
    J_rotational.resize(N_dof);
    gravity.resize(N_dof);
    mass_mat.resize(N_dof,N_dof);
    K_p.resize(6,6);
    K_d.resize(6,6);
    K_np.resize(N_dof,N_dof);
    K_nd.resize(N_dof,N_dof);
    y.resize(N_dof);
    tau_joint.resize(N_dof);
    tau_null.resize(N_dof);
    joint_nominal_config.resize(N_dof);
    error.resize(6);
    error_dot.resize(6);
    J_geom.resize(6,N_dof);
    J_p_inv.resize(N_dof,6);
    J_p_inv_w.resize(N_dof,6);
    proj.resize(N_dof,N_dof);
    
    
    //raisim activation
    auto binaryPath = raisim::Path::setFromArgv(arg);
    raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
    #if WIN32
    timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
    #endif

    //create raisim world
    world.setTimeStep(timestep);

    ///create ground and iiwa  
    ground = world.addGround(0, "brass");
    lbr_iiwa = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\lbr_iiwa\\urdf\\lbr_iiwa_pusher_collision.urdf");


    //Add ee collision property
    lbr_iiwa ->getCollisionBody("lbr_iiwa_link_ee_sphere/0").setMaterial("steel");


    //set lbr_iiwa initial configuration
    joint_nominal_config << q_1_init, q_2_init, q_3_init, q_4_init, q_5_init, q_6_init, q_7_init;
    lbr_iiwa->setGeneralizedCoordinate(joint_nominal_config);


    //get initial manipulator state
    lbr_iiwa->setName("lbr_iiwa");
    ee_frame_index = lbr_iiwa->getFrameIdxByName("lbr_iiwa_joint_7");

    update_manipulator_state();


    //Compensate gravity
    lbr_iiwa->setGeneralizedForce(gravity.e());

    //Initialize error
    error << 0,0,0,0,0,0;
    error_dot << 0,0,0,0,0,0;


    //Inverse dynamics gains
    K_p.setZero();
    K_p.block<3,3>(0,0)=stiffness*Eigen::MatrixXd::Identity(3, 3);
    K_p.block<3,3>(3,3)=0.3*stiffness*Eigen::MatrixXd::Identity(3, 3);

    K_d.setZero();
    K_d.block<3,3>(0,0)=damping*Eigen::MatrixXd::Identity(3, 3);
    K_d.block<3,3>(3,3)=0.3*damping*Eigen::MatrixXd::Identity(3, 3);

    //K_d=damping*Eigen::MatrixXd::Identity(6, 6);
    K_np=stiffness_nullspace*Eigen::MatrixXd::Identity(N_dof, N_dof);
    K_nd=damping_nullspace*Eigen::MatrixXd::Identity(N_dof, N_dof);


    //Launch server
    server.launchServer();
    while(!server.isConnected()) {raisim::MSLEEP(1);}
    server.focusOn(lbr_iiwa);

    //Create box
    box = world.addBox(box_l1, box_l2, box_l3, box_mass, box_material);

    //set box initial configuration
    box->setAppearance(box_appearance);  
    box->setPosition(box_x_init, box_y_init, box_z_init);
    box->setOrientation(box_qw_init, box_qx_init, box_qy_init, box_qz_init) ;


    //set material properties
    world.updateMaterialProp(raisim::MaterialManager(binaryPath.getDirectory() + "\\rsc\\testMaterials.xml"));
    world.setMaterialPairProp(box_material, "brass", friction, restitution, res_threshold, static_friction, static_friction_threshold_velocity);
    world.setMaterialPairProp("steel", "steel", 0.2, 0.25, 0.001);

    set_point = nullptr;
    

    //server.startRecordingVideo("snapshot_pushing.png");

}

void Raisim_pushing::spawn_obstacle(){
    //Create obstacle
    obstacle = world.addBox(0.2, 0.05, 0.2, 10000, box_material);
    obstacle->setBodyType(raisim::BodyType::STATIC);
        

    //set obstacle configuration
    update_box_state();
    obstacle->setAppearance("grey");  
    Eigen::Matrix3d box_orient_rot_90_z; 
    box_orient_rot_90_z << 0.0000000, 1.0000000,  0.0000000,
                    -1.0000000,  0.0000000,  0.0000000,
                    0.0000000,  0.0000000,  1.0000000;
    box_orient_rot_90_z = box_orientation*box_orient_rot_90_z; 
    Eigen::Vector3d obstacle_pos(0, box_l2/2+0.04, 0);
    obstacle_pos = box_position + box_orient_rot_90_z*obstacle_pos;
    obstacle->setPosition(obstacle_pos(0), obstacle_pos(1),obstacle_pos(2));
    obstacle->setOrientation(box_orient_rot_90_z);

}

void Raisim_pushing::remove_obstacle(){
    world.removeObject(obstacle);
}


void Raisim_pushing::update_manipulator_state(){
    
    //Update manipulator state
    q = lbr_iiwa->getGeneralizedCoordinate();
    q_dot = lbr_iiwa->getGeneralizedVelocity();
    lbr_iiwa->getFramePosition(ee_frame_index, ee_position);
    lbr_iiwa->getFrameOrientation(ee_frame_index, ee_orientation);
    lbr_iiwa->getFrameVelocity(ee_frame_index, ee_linear_velocity);
    lbr_iiwa->getFrameAngularVelocity(ee_frame_index, ee_angular_velocity);
    raisim::Vec<3UL> offset = {0,0,0.26};
    ee_position = ee_orientation*offset+ee_position;
    //Dynamics
    gravity = lbr_iiwa->getNonlinearities(world.getGravity());
    mass_mat=lbr_iiwa->getMassMatrix();


    //Jacobian and null space projector
    lbr_iiwa->getSparseJacobian(ee_frame_index, ee_position, J_positional);
   
    lbr_iiwa->getSparseRotationalJacobian(ee_frame_index, J_rotational); 
    
    J_geom << J_positional.e(),J_rotational.e();
    Eigen::VectorXd twist(6); twist = J_geom*q_dot.e();
    ee_linear_velocity = twist.block<3,1>(0,0);
    ee_angular_velocity = twist.block<3,1>(3,0);
    J_p_inv=J_geom.transpose()*((J_geom*J_geom.transpose()).inverse());
    J_p_inv_w=((mass_mat.e()).inverse()*J_geom.transpose()*(J_geom*(mass_mat.e()).inverse()*J_geom.transpose()).inverse());
    proj=  Eigen::MatrixXd::Identity(N_dof, N_dof)-J_p_inv_w*J_geom;
}



void Raisim_pushing::manip_motion_ctrl_1_step(Eigen::Vector3d ee_des_position_eig, Eigen::Vector3d ee_des_lin_velocity_eig, Eigen::Matrix3d ee_des_orientation_eig, Eigen::Vector3d ee_des_ang_velocity_eig){

        //Update manipulator state
        update_manipulator_state();

        //Compute orientation error
        orientation_error_matrix=ee_des_orientation_eig*ee_orientation.e().transpose();
        error_yaw = atan2(orientation_error_matrix(1,0), orientation_error_matrix(0,0));
        error_pitch = atan2(-orientation_error_matrix(2,0),sqrt((orientation_error_matrix(2,1)*orientation_error_matrix(2,1))+(orientation_error_matrix(2,2))*orientation_error_matrix(2,2)));
        error_roll = atan2(orientation_error_matrix(2,1), orientation_error_matrix(2,2));
        orientation_error_euler << error_roll, error_pitch, error_yaw;

        //Extended error vectors (position and orientation error)
        error << ee_des_position_eig - ee_position.e(), orientation_error_euler;
        error_dot << ee_des_lin_velocity_eig - ee_linear_velocity.e(), ee_des_ang_velocity_eig - ee_angular_velocity.e();
        
        //Inverse dynamics control
        y=J_p_inv_w*(K_p*error+K_d*error_dot);
        tau_joint=mass_mat.e()*y;
        tau_null=proj*(K_np*(joint_nominal_config-q.e())-(K_nd*q_dot.e()));
        for (int i = 0; i < N_dof; i++){
            if(isnan(tau_null[i])){
                tau_null[i]=0;
                std::cout << "NAN\n";
            }
        }

        //Apply control torques
        lbr_iiwa->setGeneralizedForce(tau_joint+gravity.e()+tau_null);

        //Update the state
        update_manipulator_state();

}


void Raisim_pushing::integrate_1_step(){
    raisim::MSLEEP(timestep*1000);
    server.integrateWorldThreadSafe();

}

void Raisim_pushing::update_box_state(){

    box_position = box->getComPosition();
    box_orientation=box->getRotationMatrix();
    box_linear_velocity=box ->getLinearVelocity();
    box_angular_velocity=box ->getAngularVelocity();
    box_skew << 0,-box_angular_velocity(2),0,box_angular_velocity(2),0,0,0,0,0;

}

void Raisim_pushing::get_ee_state(Eigen::Vector3d & ee_pos, Eigen::Quaterniond & ee_orient, 
                            Eigen::Vector3d & ee_lin_vel, Eigen::Vector3d & ee_ang_vel){

    update_manipulator_state();
    ee_pos = ee_position.e();
    ee_orient = ee_orientation.e();
    ee_lin_vel = ee_linear_velocity.e();
    ee_ang_vel = ee_angular_velocity.e();

}

Eigen::Vector3d Raisim_pushing::get_ee_ext_force(){
    Eigen::Vector3d ext_force;
    ext_force << 0,0,0;
    std::vector<raisim::Contact> contacts;
    contacts = lbr_iiwa->getContacts();
    //This is not so elegant...
    if(contacts.size()>0){
        ext_force = (contacts[0].getContactFrame().e().transpose() * contacts[0].getImpulse().e())/timestep;
    }
    return ext_force;
}

void Raisim_pushing::get_box_state(Eigen::Vector3d & box_pos, Eigen::Matrix3d & box_orient, 
                            Eigen::Vector3d & box_lin_vel, Eigen::Vector3d & box_ang_vel){

    update_box_state();
    box_pos = box_position;
    box_orient = box_orientation;
    box_lin_vel = box_linear_velocity;
    box_ang_vel =  box_angular_velocity;
}

void Raisim_pushing::set_force_on_box_1_step(Eigen::Vector3d force, Eigen::Vector3d point){
    //Rotate force in world frame
    force = box->getRotationMatrix()*force;

    //Apply force on box
    raisim::Vec<3> point_box={point(0), point(1), point(2)};
    raisim::Vec<3> force_box = {force(0), force(1), force(2)};
    box->setExternalForce(0,point_box ,force_box);
}


void Raisim_pushing::transform_box_2_world(Eigen::Vector3d pos_in, Eigen::Vector3d vel_in, Eigen::Vector3d & pos_out, Eigen::Vector3d & vel_out){
    update_box_state();
    vel_out = box_orientation*vel_in + box_linear_velocity + box_skew*box_orientation*pos_in;
    pos_out = box_position + box_orientation*pos_in;
} 
    
void Raisim_pushing::transform_world_2_box(Eigen::Vector3d pos_in, Eigen::Vector3d & pos_out){
    update_box_state();
    Eigen::MatrixXd T_w_b(4,4);
    Eigen::VectorXd pos_in_om(4); pos_in_om << pos_in, 1;
    Eigen::VectorXd pos_out_om(4);
    
    T_w_b.block<3,3>(0,0) = box_orientation;
    T_w_b.block<3,1>(0,3) = box_position;
    T_w_b.block<1,4>(3,0) << 0,0,0,1;
    
    pos_out_om = T_w_b.inverse()*pos_in_om;
    pos_out << pos_out_om(0), pos_out_om(1), pos_out_om(2);
}

void Raisim_pushing::add_setpoint(const std::string &name, double radius, double colorR, double colorG, double colorB, double colorA, double pos_x, double pos_y, double pos_z){
    
    set_point = server.addVisualSphere(name, radius, colorR, colorG, colorB, colorA);
    set_point->setPosition(pos_x, pos_y, pos_z);
}

void Raisim_pushing::move_setpoint(double pos_x, double pos_y, double pos_z){
    server.lockVisualizationServerMutex();
    if(set_point != nullptr){
        set_point->setPosition(pos_x, pos_y, pos_z);
    }
    server.unlockVisualizationServerMutex();

}

void Raisim_pushing::add_box_point(const std::string &name, double radius, double colorR, double colorG, double colorB, double colorA, double pos_x, double pos_y, double pos_z){
    server.lockVisualizationServerMutex();
    raisim::Visuals*new_visual = server.addVisualSphere(name, radius, colorR, colorG, colorB, colorA);
    box_points.push_back(new_visual);
    box_points.back()->setPosition(pos_x, pos_y, pos_z);
    server.unlockVisualizationServerMutex();
}

void Raisim_pushing::add_traj_point(const std::string &name, double radius, double colorR, double colorG, double colorB, double colorA, double pos_x, double pos_y, double pos_z){
    server.lockVisualizationServerMutex();
    raisim::Visuals*new_visual = server.addVisualSphere(name, radius, colorR, colorG, colorB, colorA);
    reference_traj_points.push_back(new_visual);
    reference_traj_points.back()->setPosition(pos_x, pos_y, pos_z);
    reference_traj_points.erase(reference_traj_points.begin());
    server.unlockVisualizationServerMutex();

}





Raisim_pushing::~Raisim_pushing(){
    //server.stopRecordingVideo();
    server.killServer();
}


}

/*
int main(int argc, char* argv []){

raisim_pushing::Raisim_pushing sim(argv[0]);
Eigen::Vector3d ee_pos;
Eigen::Quaterniond ee_orient;
Eigen::Vector3d ee_lin_vel;
Eigen::Vector3d ee_ang_vel;
Eigen::Vector3d zeros(0,0,0);

sim.get_ee_state(ee_pos, ee_orient, ee_lin_vel, ee_ang_vel); //Get end effector state

while(1){
    ee_pos(0) = ee_pos(0) + 0.00001;
    ee_lin_vel << 0,0,0;
    ee_lin_vel(0) = 0.00001/timestep;
    Eigen::Matrix3d Rmat(ee_orient);
    sim.manip_motion_ctrl_1_step(ee_pos, ee_lin_vel, Rmat, zeros);
    sim.integrate_1_step();

}


    return 0;
}
*/