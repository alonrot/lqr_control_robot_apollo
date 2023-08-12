#include "pole_balancing_apollo/apollo_interface.hpp"

namespace apollo_interface {

  /* ------------------------------- State class ------------------------------- */

  State::State(){
    x.resize(N_CART,0.0);
    xd.resize(N_CART,0.0);
    xdd.resize(N_CART,0.0);
  }

  /* ------------------------------- blocking functions moving the robot ------------------------------- */

  bool Go_home::move_to(SL_DJstate desired_position[N_DOFS+1], int time2finish_in_ns, std::string where2move){

    // Convert to const char*, to be able to use it on printf():
    const char * where2move_char = where2move.c_str();

    // Promt to user:
    int ans = 999;
    printf("\n    Enter 1 to move to %s , or anything else to abort ...\n\n",where2move_char);
    while (ans == 999) {
      if (!get_int("Enter: ",ans,&ans))
        return false;
    }
    
    if (ans != 1)
      return false;
    printf("\n    Moving to %s ...\n\n",where2move_char);
    
    // Going to the initial position:
    bool is_reached = FALSE;
    while (!is_reached)
    {
     // Checking whether any other task is running:
     if (strcmp(current_task_name,NO_TASK) != 0)
       taskDelay(ns2ticks(time2finish_in_ns));
     else
       is_reached = go_target_wait_ID(desired_position);
    }
    
    printf("\n    %s reached!\n\n",where2move_char);
    return true;

  }

  void Go_home::read_default_position(SL_DJstate desired_position[N_DOFS+1]){

    for (int i=1; i<=N_DOFS; i++){
      desired_position[i].th = joint_default_state[i].th;
      desired_position[i].thd = 0;
      desired_position[i].thdd = 0;
      desired_position[i].uff = 0;
    }
  }

  Go_home_pole_balancing::Go_home_pole_balancing(bool use_head){
    this->use_head = use_head;
  }

  void Go_home_pole_balancing::add_position_home(SL_DJstate desired_position[N_DOFS+1]){

    desired_position[R_SFE].th  =  0.081;
    desired_position[R_SAA].th  = -0.013;
    desired_position[R_HR].th   =  0.199;
    desired_position[R_EB].th   =  1.499;
    desired_position[R_WR].th   = -0.054;
    desired_position[R_WFE].th  = -0.323;
    desired_position[R_WAA].th  =  0.012;
    desired_position[R_FR].th   =  0;    
  }

  void Go_home_pole_balancing::add_position_hand_closed(SL_DJstate desired_position[N_DOFS+1]){

    desired_position[R_LF].th   = 2.43;
    desired_position[R_MF].th   = 2.43;
    desired_position[R_RF].th   = 2.43;
  }

  void Go_home_pole_balancing::add_position_head(SL_DJstate desired_position[N_DOFS+1]){

    double deltaX, deltaY, deltaZ;
    double theta_pan_ini,theta_tilt_ini;

    // Head position: looking at end-effector position
    deltaX = cart_state[1].x[0] - joint_origin_pos[B_HR][0];
    deltaY = cart_state[1].x[1] - joint_origin_pos[B_HR][1];
    deltaZ = 0.3;
    theta_pan_ini  =  atan2_save(deltaY,deltaX);
    theta_tilt_ini = -atan2_save(Norm(deltaX,deltaY),deltaZ);

    // Add to the desired position:    
    desired_position[B_HN].th = cos(theta_pan_ini)*theta_tilt_ini;
    desired_position[B_HT].th = sin(theta_pan_ini)*theta_tilt_ini;
    desired_position[B_HR].th = theta_pan_ini;
  }

  bool Go_home_pole_balancing::move_to_initial_position()
  {
    bool  success;
    int   prepare = 0;
    int   time2move_in_ns = 3000000000;
    SL_DJstate * position_desired = (SL_DJstate *)my_calloc(N_DOFS+1,sizeof(SL_DJstate),MY_STOP);
    read_default_position(position_desired);
    add_position_home(position_desired);
    
    get_int("Moving the arm to the initial position:\n    [0] The pole is already placed on its hand\n    [1] The pole must be placed on Apollo's hand  \nYour choice: ",prepare,&prepare);
    
    // First go home, and then close the hand:
    if(prepare){

      // Move first to the home position:
      success = move_to(position_desired,time2move_in_ns,"home position");
      if (!success) 
        return false;
    
      // Add hand closed position:
      add_position_hand_closed(position_desired);

      // Add head position:
      if (this->use_head)
        add_position_head(position_desired);

      // Move to hand-closed position:
      success = move_to(position_desired,time2move_in_ns,"hand-closed position");

    }
    else{

      // Add hand closed position:
      add_position_hand_closed(position_desired);

      // Add head position:
      if (this->use_head)
        add_position_head(position_desired);

      // Move first to the home position:
      success = move_to(position_desired,time2move_in_ns,"home with hand closed");
    }

    // Using free() because it is allocated with malloc
    free(position_desired);

    return success;
  }



  /* ------------------------------- Target_state_in_robot_frame class ------------------------------- */  

  Target_state_in_robot_frame::Target_state_in_robot_frame(const std::vector<double> cartesian_offset, double time_step, double plane_angle) {
    
    for(int i=0;i<3;i++){
      this->previous_position[i]      = cartesian_offset[i];
      this->previous_velocity[i]      = 0;
      this->previous_acceleration[i]  = 0;
    }

    this->time_step   = time_step;
    this->plane_angle = plane_angle;
    this->initial_condition_euler_integration = true;
  }

  State Target_state_in_robot_frame::get_from_endeff_acceleration(double endeffector_acceleration)
  {

    State target_state;
    
    // Target acceleration for the end effector, only for x and y:
    target_state.xdd[0] =  endeffector_acceleration * cos(-this->plane_angle);
    target_state.xdd[1] = -endeffector_acceleration * sin(-this->plane_angle);

    if (this->initial_condition_euler_integration){
      for(int i=0;i<3;++i)
        target_state.x[i]   = this->previous_position[i];
        // target_state.xd[i] = 0.0; // already
        // target_state.xdd[i] = u[i]; // already
      this->initial_condition_euler_integration = false;
    }
    else{
      // Target velocity and position, only for x and y:
      for(int i=0;i<3;i++){
        target_state.xd[i]  = this->previous_velocity[i] + ( this->previous_acceleration[i] + target_state.xdd[i] ) * this->time_step/2.0;
        target_state.x[i]   = this->previous_position[i] + ( this->previous_velocity[i] + target_state.xd[i] ) * this->time_step/2.0;
      }
    }
    
    // Saving state for next iteration, only for x and y:
    for(int i=0;i<3;i++){
      this->previous_acceleration[i]  = target_state.xdd[i];
      this->previous_velocity[i]      = target_state.xd[i]; 
      this->previous_position[i]      = target_state.x[i];
    }

    bool verbo = false;

    if (verbo){
      printf("endeffector_acceleration = %f\n",endeffector_acceleration);
      for(int i=0;i<3;++i)
        printf("target_state.x[%i] = %f\n",i,target_state.x[i]);
      for(int i=0;i<3;++i)
        printf("target_state.xd[%i] = %f\n",i,target_state.xd[i]);
      for(int i=0;i<3;++i)
        printf("target_state.xdd[%i] = %f\n",i,target_state.xdd[i]);
    }

    return target_state;
  }


  /* ------------------------------- Inverse_kinematics_and_dynamics class ------------------------------- */  

  Inverse_kinematics_and_dynamics::Inverse_kinematics_and_dynamics(int filter_order, int filter_cutoff, double time_step){

    // Initialize SL filters:
    this->fthdd = (Filter *)my_calloc(n_dofs+1,sizeof(Filter),MY_STOP);
    for (int i=1; i<=n_dofs; ++i) 
      for (int j=0; j<=filter_order; ++j)
	      this->fthdd[i].raw[j] = fthdd[i].filt[j] = 0;
    for (int i=1; i<=n_dofs; ++i) 
      this->fthdd[i].cutoff = filter_cutoff;

    // Initialize joint target state, needed for the IK (all the fields are zeroed by my_calloc):
    this->joints_target = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);
    
    // Preparing cartesian target for IK: [xd, yd, zd, ad, bd, gd]: make sure that the irrelevant part of cart is set to zero
    this->cart = my_vector(1,N_ENDEFFS*6); 
    
    // Preparing for the inverse kinematics of the endeffector:
      // Which dimensions of endeffector to be included in inverse kinematics? All of right arm.
    this->cstatus = my_ivector(1,N_ENDEFFS*6);
    for(int i=1;i<=6;i++) 
      this->cstatus[i]=1;

    // Get the i9ndex of the joints that need to be frozen, because they are unused:
    for (int i=L_SFE; i<=N_ROBOT_DOFS; ++i){
      if ( i != B_HN && i != B_HT && i != B_HR) {
	      this->joints_to_freeze.push_back(i);
      }
    }

    // Time step:
    this->time_step = time_step;

   }

   Inverse_kinematics_and_dynamics::~Inverse_kinematics_and_dynamics(){
    // Free memory:
    free(this->fthdd);
    free(this->joints_target);
    // my_free_ivector(this->cstatus,1,N_ENDEFFS*6);
    // my_free_vector(this->cart,1,N_ENDEFFS*6);
    free(this->cstatus);
    free(this->cart);

   }

  bool Inverse_kinematics_and_dynamics::update_desired_state_arm(State endeffector_target){

    int i;
    double aux;
    bool verbo = false;

    // Read the target state in the cartesian space of the end-effector (only velocities are needed):
    // The remaining elements of cart[] stay unchanged (and should always be equal to zero).
    this->cart[1] = endeffector_target.xd[0];
    this->cart[2] = endeffector_target.xd[1];

    if(verbo)
      for (i=1; i<=N_ENDEFFS*6; ++i)
        printf("@update_desired_state_arm: this->cart[%i] = %f\n",i,this->cart[i]);

    // Read the target state in the joint space of the arm:
    for (i=1; i<=R_WAA; ++i){
      this->joints_target[i].th = joint_des_state[i].th;
    }

    if(verbo){
      for (i=1; i<=N_ROBOT_DOFS; ++i){
        printf("@update_desired_state_arm: this->joints_target[%i].th   = %f\n",i,this->joints_target[i].th);
        printf("@update_desired_state_arm: this->joints_target[%i].thd  = %f\n",i,this->joints_target[i].thd);
        printf("@update_desired_state_arm: this->joints_target[%i].thdd = %f\n",i,this->joints_target[i].thdd);
        printf("@update_desired_state_arm: this->joints_target[%i].uff  = %f\n",i,this->joints_target[i].uff);
      }
    }

    if(verbo){
      for (i=1; i<=N_ENDEFFS*6; ++i)
        printf("@update_desired_state_arm: this->cart[%i] = %f\n",i,this->cart[i]);
      for (i=1; i<=N_ENDEFFS*6; ++i)
        printf("@update_desired_state_arm: this->cstatus[%i] = %i\n",i,this->cstatus[i]);
    }

    // Call inverse kinematics:
    if (!inverseKinematics(this->joints_target,endeff,joint_opt_state,this->cart,this->cstatus,this->time_step)) {
      freeze();
      printf("IK failed! Robot has been frozen\n");
      return false;
    }

    if(verbo)
      printf("@update_desired_state_arm: After applying IK:\n");

    if(verbo){
      for (i=1; i<=N_ROBOT_DOFS; ++i){
        printf("@update_desired_state_arm: this->joints_target[%i].th = %f\n",i,this->joints_target[i].th);
        printf("@update_desired_state_arm: this->joints_target[%i].thd = %f\n",i,this->joints_target[i].thd);
        printf("@update_desired_state_arm: this->joints_target[%i].thdd = %f\n",i,this->joints_target[i].thdd);
        printf("@update_desired_state_arm: this->joints_target[%i].uff = %f\n",i,this->joints_target[i].uff);
      }
    }

    for (i=1; i<=R_WAA; ++i) {
        aux = (this->joints_target[i].thd-joint_des_state[i].thd)*(double)task_servo_rate;
        this->joints_target[i].thdd  = filt(aux,&(this->fthdd[i])); 
        joint_des_state[i].thdd = this->joints_target[i].thdd;
        joint_des_state[i].thd  = this->joints_target[i].thd;
        joint_des_state[i].th   = this->joints_target[i].th;
        if (joint_des_state[i].th > joint_range[i][MAX_THETA]) {
            joint_des_state[i].th = joint_range[i][MAX_THETA];
            joint_des_state[i].thd = 0.0;
            joint_des_state[i].thdd = 0.0;
        }
        if (joint_des_state[i].th < joint_range[i][MIN_THETA]) {
            joint_des_state[i].th = joint_range[i][MIN_THETA];
            joint_des_state[i].thd = 0.0;
            joint_des_state[i].thdd = 0.0;
        }

        if(verbo){
          printf("@update_desired_state_arm: aux = %f\n",aux);
          printf("@update_desired_state_arm: this->joints_target[%i].thdd = %f\n",i,this->joints_target[i].thdd);
          printf("@update_desired_state_arm: joint_des_state[%i].thdd = %f\n",i,joint_des_state[i].thdd);
          printf("@update_desired_state_arm: joint_des_state[%i].thd  = %f\n",i,joint_des_state[i].thd);
          printf("@update_desired_state_arm: joint_des_state[%i].th   = %f\n",i,joint_des_state[i].th);
          printf("@update_desired_state_arm: joint_des_state[%i].uff  = %f\n",i,joint_des_state[i].uff);
        }
    }

    // Freeze inactive joints:
    for(i=0;i<this->joints_to_freeze.size();i++){
      joint_des_state[this->joints_to_freeze[i]].th   = joint_state[joints_to_freeze[i]].th;
      joint_des_state[this->joints_to_freeze[i]].thd  = 0;
      joint_des_state[this->joints_to_freeze[i]].thdd = 0;
    }

    if(verbo)
      printf("@update_desired_state_arm: \n\n");

    return true;

  }

  void Inverse_kinematics_and_dynamics::update_desired_state_head(void)
  {
    /* Obtain head's target position */
    // Head position:
    double deltaX = cart_state[1].x[_X_] - joint_origin_pos[B_HR][_X_];
    double deltaY = cart_state[1].x[_Y_] - joint_origin_pos[B_HR][_Y_];
    double deltaZ = 0.3;

    // Computing the pan angle of the head:
    // theta_pan  =  atan2(deltaX,deltaY);
    double theta_pan  =  atan2_save(deltaY,deltaX);

    // Computing the tilt angle of the head:
    // theta_tilt = -atan2(deltaZ,Norm(deltaX,deltaY));
    double theta_tilt = -atan2_save(Norm(deltaX,deltaY),deltaZ);

    /* Compute head's IK */
    double auxd;
    int ind[3];
    double new_head_angle[3];
    double sec = 1.0;

    // Some coordinate transformations are needed:
    ind[0] = B_HN;
    ind[1] = B_HT;
    ind[2] = B_HR;

    new_head_angle[0] = cos(theta_pan)*theta_tilt;
    new_head_angle[1] = sin(theta_pan)*theta_tilt;
    new_head_angle[2] = theta_pan; 

    // Computing the desired kinematics for the head:
    for(int k=0;k<3;++k)
    {
      auxd = ( new_head_angle[k] - joint_des_state[ind[k]].th )*(double)task_servo_rate;
      joint_des_state[ind[k]].thdd = ( auxd - joint_des_state[ind[k]].thd )*(double)task_servo_rate;
      joint_des_state[ind[k]].thd  = auxd;
      joint_des_state[ind[k]].th   = new_head_angle[k];

      // Saturation:
      if ( joint_des_state[ind[k]].th > sec*joint_range[ind[k]][MAX_THETA] )
      {
        joint_des_state[ind[k]].th = sec*joint_range[ind[k]][MAX_THETA];
        joint_des_state[ind[k]].thd = 0.0;
        joint_des_state[ind[k]].thdd = 0.0;
      }
      if ( joint_des_state[ind[k]].th < sec*joint_range[ind[k]][MIN_THETA] )
      {
        joint_des_state[ind[k]].th = sec*joint_range[ind[k]][MIN_THETA];
        joint_des_state[ind[k]].thd = 0.0;
        joint_des_state[ind[k]].thdd = 0.0;
      }
    }
  }

  void Inverse_kinematics_and_dynamics::update_uff(void)
  {
    // Adds torque to joint_des_state[i].uff
    SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);
  }


  /* ------------------------------- function that returns current jacobian transpose ------------------------------- */  

  static void _get_jacobian_transpose(SL_DJstate *target, Matrix jacobian_transpose){

    // MY_MATRIX : create matrix on the stack with name provided in the first input argument:
    MY_MATRIX(Jac,1,6*N_ENDEFFS,1,N_DOFS);
    MY_MATRIX(Jreal,1,6*N_ENDEFFS,1,N_DOFS);
    MY_MATRIX(J_pinv,1,N_DOFS,1,6*N_ENDEFFS);
    MY_IVECTOR(ind,1,6*N_ENDEFFS);
    MY_MATRIX(local_link_pos_des,0,N_LINKS,1,3);
    MY_MATRIX(local_joint_cog_mpos_des,0,N_DOFS,1,3);
    MY_MATRIX(local_joint_origin_pos_des,0,N_DOFS,1,3);
    MY_MATRIX(local_joint_axis_pos_des,0,N_DOFS,1,3);
    MY_MATRIX_ARRAY(local_Alink_des,1,4,1,4,N_LINKS+1);
    MY_MATRIX_ARRAY(local_Adof_des,1,4,1,4,N_DOFS+1);
    double condnr;
    double condnr_cutoff = 70.0;  // this corresponds to condnr_cutoff^2 in inverse space

    // Compute the Jacobian  
    linkInformationDes(target,&base_state,&base_orient,endeff,
                      local_joint_cog_mpos_des,
                      local_joint_axis_pos_des,
                      local_joint_origin_pos_des,
                      local_link_pos_des,
                      local_Alink_des,
                      local_Adof_des);
    
    jacobian(local_link_pos_des,local_joint_origin_pos_des,local_joint_axis_pos_des,Jac);

    int count = 0;
    for (int i=1; i<=6*N_ENDEFFS; ++i) {
      if( i==1 || i==2 || i==3 ) { // DOFs involved in the cartesian feedback control loop: position [x,y,z] from right hand
        ++count;
        ind[count] = i;
      }
    }

    // The Jacobian that is actually needed, i.e., only the constraint rows
    mat_zero(Jreal);
    for (int i=1; i<=count; ++i)
      for (int j=1; j<=N_DOFS; ++j)
        Jreal[i][j] = Jac[ind[i]][j];

    // Transpose:
    mat_trans(Jreal,jacobian_transpose);

  }


  /* ------------------------------- final control ------------------------------- */  

  Cartesian_feedback_control_loop::Cartesian_feedback_control_loop( bool use_forg_factor, int win_time, double kp, 
                                                                    double kd, double ki, double time_step){

    // Initialize to 1 ind_win_per_joint:
    this->ind_win_per_joint.resize(R_WAA+1,0);
    for(int i=1;i<=R_WAA;++i)
      this->ind_win_per_joint[i] = 1;

    
    for(int i=1;i<=3;i++){
      this->KP_pose[i+1]=kp;
      this->KD_pose[i+1]=kd;
      this->KI_pose[i+1]=ki;
    }
    
    this->use_forg_factor = use_forg_factor;
    this->win_time = win_time;
    this->win_length = win_time*((int)task_servo_rate);
    this->integrator_window = my_matrix(1,R_WAA,1,win_length);
    this->ufb_pose_KP = my_vector(1,N_DOFS);
    this->ufb_pose_KD = my_vector(1,N_DOFS);
    this->ufb_pose_KI = my_vector(1,N_DOFS);
    this->time_step = time_step;
  }

  Cartesian_feedback_control_loop::~Cartesian_feedback_control_loop(){
    // SL my_free_matrix and my_free_vector crashed:
    free(this->integrator_window);
    free(this->ufb_pose_KP);
    free(this->ufb_pose_KD);
    free(this->ufb_pose_KI);
    
  }


  // this update joint_des_state
  void Cartesian_feedback_control_loop::update_arm_uff(State endeffector_target, const Matrix jacobian_transpose, bool dbg_integrator){

    std::vector<double> int_state_pose;
    int_state_pose.resize(R_WAA+1,0.0);
    
    // Compute the ufb contribution of each joint i, to be added to uff
    for (int i=1; i<=R_WAA; ++i)
    {
      // Initialization: Set to zero
      this->ufb_pose_KP[i] = 0.0;
      this->ufb_pose_KD[i] = 0.0;

      if (!this->use_forg_factor)
        this->ufb_pose_KI[i] = 0.0;
      
      int_state_pose[i] = 0.0;

      // Compute ufb for each joint
      for (int j=1; j<=3; ++j){
        this->ufb_pose_KP[i]  += jacobian_transpose[i][j]*KP_pose[j]*( endeffector_target.x[j-1] - cart_state[RIGHT_HAND].x[j] );
        this->ufb_pose_KD[i]  += jacobian_transpose[i][j]*KD_pose[j]*( endeffector_target.xd[j-1] - cart_state[RIGHT_HAND].xd[j] );
        int_state_pose[i]     += jacobian_transpose[i][j]*KI_pose[j]*( endeffector_target.x[j-1] - cart_state[RIGHT_HAND].x[j] )*this->time_step;
      }

      // Update ufb integrator contribution depending on whether we want to use forgetting factor or not: 
      if (this->use_forg_factor)
      {
        // Forgetting factor time-window:
        this->ufb_pose_KI[i] -= this->ufb_pose_KI[i]/win_length;
        this->ufb_pose_KI[i] += int_state_pose[i];
      }
      else
      {
        int ind_win;
        // Standard time-window (similar) for each joint i:
        if (dbg_integrator)
          ind_win = this->ind_win_per_joint[1];
        else
          ind_win = this->ind_win_per_joint[i];

        this->integrator_window[i][ind_win] = int_state_pose[i];
        for(int c=1;c<=this->win_length;++c)
          ufb_pose_KI[i] += this->integrator_window[i][c];

        // Increment the index for joint i:
        ind_win = ind_win % this->win_length + 1;
        
        if (dbg_integrator)
          this->ind_win_per_joint[1] = ind_win;
        else
          this->ind_win_per_joint[i] = ind_win;
      }
      
    	// The uff is read in the motor servo trhough the shared memory, thus we add the torque to this variable:
    	joint_des_state[i].uff += this->ufb_pose_KP[i] + this->ufb_pose_KD[i] + this->ufb_pose_KI[i];

    }
  }


  /* ------------------------------- Measure endeffector state ------------------------------- */  

  Measure_endeffector_cartesian_state::Measure_endeffector_cartesian_state(double noise_std, int sampling_steps, int delay_time_steps){

    // Initialize end-effector sensor:
    for(int i=0;i<N_CART;++i){
      this->sensor_endeff.x[i]    = std::make_shared<FakeSensor>(noise_std,sampling_steps,delay_time_steps);
      this->sensor_endeff.xd[i]   = std::make_shared<FakeSensor>(noise_std,sampling_steps,delay_time_steps);
      this->sensor_endeff.xdd[i]  = std::make_shared<FakeSensor>(noise_std,sampling_steps,delay_time_steps);
    }

  }

  void Measure_endeffector_cartesian_state::update(){
    // Copy over:
    for(int i=0; i<N_CART ; ++i){
      this->endeffector_state_measured.x[i]   = this->sensor_endeff.x[i]->measure(cart_state[1].x[i+1]);
      this->endeffector_state_measured.xd[i]  = this->sensor_endeff.xd[i]->measure(cart_state[1].xd[i+1]);
      this->endeffector_state_measured.xdd[i] = this->sensor_endeff.xdd[i]->measure(cart_state[1].xdd[i+1]);
    }
  }

  CartesianState Measure_endeffector_cartesian_state::update_and_get(){

    // First update:
    this->update();

    // Copy over:
    CartesianState endeffector_state;
    for(int i=0;i<N_CART;++i){
      endeffector_state.x[i] = this->endeffector_state_measured.x[i];
      endeffector_state.xd[i] = this->endeffector_state_measured.xd[i];
      endeffector_state.xdd[i] = this->endeffector_state_measured.xdd[i];
    }

    return endeffector_state;
  }


  /* ------------------------------- standalone functions ------------------------------- */

  void read_cartesian_offset(std::vector<double> & cartesian_offset){
    cartesian_offset[0] = cart_state[1].x[_X_];
    cartesian_offset[1] = cart_state[1].x[_Y_];
    cartesian_offset[2] = cart_state[1].x[_Z_];
  }

  /* ------------------------------- Stop Robot class ------------------------------- */

  StopApollo::StopApollo( bool record_data, int duration_task, std::vector<double> endeff_box_limits){

    this->record_data         = record_data;
    this->duration_task       = duration_task;
    this->start_time          = start_time;
    this->endeff_box_limits   = endeff_box_limits;

    this->current_task_time = 0.0;

  }

  bool StopApollo::within_time(double task_servo_time){

    // Update elapsed time within this task:
    this->current_task_time = task_servo_time - this->start_time;

    // STOP CRITERIA: Finish the task when the total time is consumed:
    if ( this->current_task_time > (double)this->duration_task ){

      freeze();

      #ifdef __XENO__
        rt_printf("\n\n    Task is finisehd!\n\n");
      #else
        printf("\n\n    Task is finisehd!\n\n");
      #endif

      if(this->record_data){
        stopcd();
        sendCommandLineCmd((char *)"saveData");
      }

      return false;
    }
    
    return true;
  }

  bool StopApollo::within_task_space_limits(std::vector<double> endeff_pos_plane)
  {
    // We check if the robot's end-effector gets out of the "box":
    if( fabs(endeff_pos_plane[0]) > this->endeff_box_limits[0] || 
        fabs(endeff_pos_plane[1]) > this->endeff_box_limits[1] || 
        fabs(endeff_pos_plane[2]) > this->endeff_box_limits[2]   ){

        freeze();

        #ifdef __XENO__
          rt_printf("The robot's end-effector space limits have been reached\n");
        #else
          printf("The robot's end-effector space limits have been reached\n");
        #endif

        if(this->record_data){
          stopcd();
          sendCommandLineCmd((char *)"saveData");
        }

      return false;
      }

    return true;
  }

  bool StopApollo::within_joint_space_limits(SL_Jstate * joint_state, double joint_range[][4]){

    for (int i=1; i<=R_WAA; ++i)
    {

      if( joint_state[i].th > joint_range[i][MAX_THETA] || joint_state[i].th < joint_range[i][MIN_THETA]){

        freeze();
        
        #ifdef __XENO__
          rt_printf("Joint %i is out of the limits\n",i);
          rt_printf("\n\n    Task is finisehd!\n\n");
        #else
          printf("Joint %i is out of the limits\n",i);
          printf("\n\n    Task is finisehd!\n\n");
        #endif


        if(this->record_data){
          stopcd();
          sendCommandLineCmd((char *)"saveData");
        }

        return false;
      }
    }

    return true;

  }


  /* ------------------------------------- control interface for end user ------------------------------------- */

  bool init_control(bool use_forg_factor, int window_time, double kp, double kd, double ki, 
                    int filter_order, int filter_cutoff, double time_step, double plane_angle){

    // Read current end-effector state, treated as a constant offset from now on:
    std::vector<double> cartesian_offset;
    cartesian_offset.resize(3);
    read_cartesian_offset(cartesian_offset);

    // Construct object that stores the end-effector's target, used in IK:
    target_in_robot_frame = std::make_shared<Target_state_in_robot_frame>(cartesian_offset,time_step,plane_angle);

    // Construct object to add uff torque to each joint from Apollo's Inverse Kinematics and Inverse Dynamics (IKD):
    apollo_IKD = std::make_shared<Inverse_kinematics_and_dynamics>(filter_order,filter_cutoff,time_step);

    // Construct object to add uff torque to each joint from Apollo's end-effector feedback control loop:
    apollo_endeff_feedback = std::make_shared<Cartesian_feedback_control_loop>(use_forg_factor,window_time,kp,kd,ki,time_step);

  }

  
  bool apply_control(double control, bool use_head, bool dbg_integrator){

    // Get target state in robot frame:
    State endeffector_target = target_in_robot_frame->get_from_endeff_acceleration(control);
    
    // Apply inverse kinematics and dynamics (update joint_des_state) and then update uff:
    if( !apollo_IKD->update_desired_state_arm(endeffector_target) )
      return false;

    if(use_head)
      apollo_IKD->update_desired_state_head();

    apollo_IKD->update_uff();

    // Jacobian transpose:
    MY_MATRIX(jacobian_transpose,1,N_DOFS,1,6*N_ENDEFFS);
    _get_jacobian_transpose(apollo_IKD->joints_target,jacobian_transpose);

    // Add uff to apollo's joints, from the endeffector cartesian feedback control loop:
    apollo_endeff_feedback->update_arm_uff(endeffector_target,jacobian_transpose,dbg_integrator);

    return true;

  }


  /* -------------------------------- stopping options interface for end user -------------------------------- */

  void init_stop_criterions(bool record_data, int duration_task, std::vector<double> endeff_box_limits){

    // Construct class:
    stop_robot = std::make_shared<StopApollo>(record_data,duration_task,endeff_box_limits);

  }

  bool check_stopping_criterions(std::vector<double> endeff_pos_in_plane){

    // Stop the robot if the maximum time is elapsed:
    if( !stop_robot->within_time(task_servo_time) )
      return false;

    // Stop the robot if the end-effector is out of the safety box:
    if( !stop_robot->within_task_space_limits(endeff_pos_in_plane) )
      return false;

    // Stop the robot if the joints are out of the limits:
    if( !stop_robot->within_joint_space_limits(joint_state,joint_range) )
      return false;

  return true;
    
  }

  /* -------------------------------- Classes for testing purposes -------------------------------- */


} // namespace apollo_interface

  /* -------------------------------- Record data -------------------------------- */

namespace record_SL_data{

  Rec_LQRPoleBalancing::Rec_LQRPoleBalancing( std::shared_ptr<parameters::YAML_data_share> estimate_states_pars,
                                              std::shared_ptr<parameters::YAML_data_share> measurements_pars,
                                              std::shared_ptr<parameters::YAML_data_share> control_pars)
                                              : estimate_states_pars(estimate_states_pars),
                                                measurements_pars(measurements_pars),
                                                control_pars(control_pars){

    /* From estimate states class */
      // Initialize:
        states.resize(5,0.0);
  
        // Add to SL collector:
        addVarToCollect((char *)&(this->states[0]),"states[PHI]","[rad]",DOUBLE,TRUE);
        addVarToCollect((char *)&(this->states[1]),"states[PHID]","[rad/s]",DOUBLE,TRUE);
        addVarToCollect((char *)&(this->states[2]),"states[S]","[m]",DOUBLE,TRUE);
        addVarToCollect((char *)&(this->states[3]),"states[SD]","[m/s]",DOUBLE,TRUE);
        addVarToCollect((char *)&(this->states[4]),"states[INT_]","[m*s]]",DOUBLE,TRUE);

    /* From measurements class */
      // Initialize:
      this->measured_pole_angle = 0.0;
      this->visualization_pole_angle = 0.0;

      // Add to SL collector:
      addVarToCollect((char *)&(this->measured_pole_angle),"y[PHI]","[rad]",DOUBLE,TRUE);
      addVarToCollect((char *)&(this->visualization_pole_angle),"y_vis[PHI]","[rad]",DOUBLE,TRUE);

    /* From Feedbcak control class */
      // Initialize:
      this->u = 0.0;

      // Add to SL collector:
      addVarToCollect((char *)&(this->u),"u","[m/s2]",DOUBLE,TRUE);

  }

  void
  Rec_LQRPoleBalancing::update_readings(){
    std::vector<double> states_tmp = this->estimate_states_pars->get<std::vector<double>>("states");
    for(int i=0;i<5;++i)
      this->states[i] = states_tmp[i];
    this->measured_pole_angle = this->measurements_pars->get<double>("y[PHI]");
    this->visualization_pole_angle = this->measurements_pars->get<double>("y_vis[PHI]");
    this->u = this->control_pars->get<double>("u");
  }

} // namespace record_SL_data

