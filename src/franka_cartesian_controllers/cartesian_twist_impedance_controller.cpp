// This code was derived from franka_example controllers
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license.
// Current development and modification of this code by Nadia Figueroa (MIT) 2021.

#include <cartesian_twist_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <pseudo_inversion.h>
#include <hardware_interface/joint_command_interface.h>

namespace franka_interactive_controllers {

bool CartesianTwistImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_desired_twist_ = node_handle.subscribe(
      "/cartesian_impedance_controller/desired_twist", 20, &CartesianTwistImpedanceController::desiredTwistCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_desired_cartesian_stiffness_ = node_handle.subscribe(
      "/cartesian_impedance_controller/desired_cartesian_stiffness",
      20, &CartesianTwistImpedanceController::desiredCartesianStiffnessCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
  sub_desired_nullspace_stiffness_ = node_handle.subscribe(
      "/cartesian_impedance_controller/desired_nullspace_stiffness",
      20, &CartesianTwistImpedanceController::desiredNullspaceStiffnessCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_desired_external_tool_compensation_ = node_handle.subscribe(
      "/cartesian_impedance_controller/desired_external_tool_compensation",
      20, &CartesianTwistImpedanceController::desiredExternalToolCompensationCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Getting ROSParams
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianTwistImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianTwistImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // Initialize variables for tool compensation from yaml config file
  activate_tool_compensation_ = true;
  tool_compensation_force_.setZero();
  std::vector<double> external_tool_compensation;
  // tool_compensation_force_ << 0.46, -0.17, -1.64, 0, 0, 0;  //read from yaml
  if (!node_handle.getParam("external_tool_compensation", external_tool_compensation) || external_tool_compensation.size() != 6) {
      ROS_ERROR(
          "CartesianTwistImpedanceController: Invalid or no external_tool_compensation parameters provided, "
          "aborting controller init!");
      return false;
    }
  for (size_t i = 0; i < 6; ++i) 
    tool_compensation_force_[i] = external_tool_compensation.at(i);
  ROS_INFO_STREAM("External tool compensation force: " << std::endl << tool_compensation_force_);

  // Initialize variables for nullspace control from yaml config file
  q_d_nullspace_.setZero();
  std::vector<double> q_nullspace;
  if (node_handle.getParam("q_nullspace", q_nullspace)) {
    q_d_nullspace_initialized_ = true;
    if (q_nullspace.size() != 7) {
      ROS_ERROR(
        "CartesianTwistImpedanceController: Invalid or no q_nullspace parameters provided, "
        "aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < 7; ++i) 
      q_d_nullspace_[i] = q_nullspace.at(i);
    ROS_INFO_STREAM("Desired nullspace position (from YAML): " << std::endl << q_d_nullspace_);
  }

  nullspace_stiffness_target_.setIdentity();
  nullspace_damping_target_.setIdentity();
  std::vector<double> nullspace_stiffness_target_yaml;
  if (!node_handle.getParam("nullspace_stiffness_target", nullspace_stiffness_target_yaml) || nullspace_stiffness_target_yaml.size() != 7) {
    ROS_ERROR(
      "CartesianTwistImpedanceController: Invalid or no nullspace_stiffness_target_yaml parameters provided, "
      "aborting controller init!");
    return false;
  }
  for (int i = 0; i < 7; i ++) {
    nullspace_stiffness_target_(i,i) = nullspace_stiffness_target_yaml[i];
  }
  for (int i = 0; i < 7; i ++) {
    nullspace_damping_target_(i,i) = 2.0 * sqrt(nullspace_stiffness_target_yaml[i]);
  }
  ROS_INFO_STREAM("nullspace_stiffness_target_: " << std::endl <<  nullspace_stiffness_target_);
  ROS_INFO_STREAM("nullspace_damping_target_: " << std::endl <<  nullspace_damping_target_);
  
  // Initialize stiffness
  cartesian_stiffness_target_.setIdentity();
  cartesian_damping_target_.setIdentity();
  std::vector<double> cartesian_stiffness_target_yaml;
  if (!node_handle.getParam("cartesian_stiffness_target", cartesian_stiffness_target_yaml) || cartesian_stiffness_target_yaml.size() != 6) {
    ROS_ERROR(
      "CartesianTwistImpedanceController: Invalid or no cartesian_stiffness_target_yaml parameters provided, "
      "aborting controller init!");
    return false;
  }
  for (int i = 0; i < 6; i ++) {
    cartesian_stiffness_target_(i,i) = cartesian_stiffness_target_yaml[i];
  }
  // Damping ratio = 1
  for (int i = 0; i < 6; i ++) {
    cartesian_damping_target_(i,i) = 2.0 * sqrt(cartesian_stiffness_target_yaml[i]);
  }
  ROS_INFO_STREAM("cartesian_stiffness_target_: " << std::endl <<  cartesian_stiffness_target_);
  ROS_INFO_STREAM("cartesian_damping_target_: " << std::endl <<  cartesian_damping_target_);

  // Getting libranka control interfaces
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianTwistImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianTwistImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianTwistImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianTwistImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianTwistImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianTwistImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Getting Dynamic Reconfigure objects
  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_interactive_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianTwistImpedanceController::complianceParamCallback, this, _1, _2));

  // Initializing variables
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  velocity_d_.setZero();

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // Parameters for goto_home at initialization!!
  _goto_home = false;

  // Parameters for jointDS controller (THIS SHOULD BE IN ANOTHER SCRIPT!! DS MOTION GENERATOR?)
  q_home_ << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;
  jointDS_epsilon_  = 0.05;
  dq_filter_params_ = 0.555;

  A_jointDS_home_ = Eigen::MatrixXd::Identity(7, 7);
  A_jointDS_home_(0,0) = 10; A_jointDS_home_(1,1) = 10; A_jointDS_home_(2,2) = 10;
  A_jointDS_home_(3,3) = 10; A_jointDS_home_(4,4) = 15; A_jointDS_home_(5,5) = 15;
  A_jointDS_home_(6,6) = 15;
  ROS_INFO_STREAM("A (jointDS): " << std::endl <<  A_jointDS_home_);

  // Parameters for joint PD controller
  // Ideal gains for Joint Impedance Controller
  // k_gains: 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0
  // d_gains: 50.0, 50.0, 50.0, 20.0, 20.0, 20.0, 10.0

  // Gains for P error stiffness term
  k_joint_gains_ = Eigen::MatrixXd::Identity(7, 7);
  k_joint_gains_(0,0) = 500; k_joint_gains_(1,1) = 500; k_joint_gains_(2,2) = 500;
  k_joint_gains_(3,3) = 500; k_joint_gains_(4,4) = 500; k_joint_gains_(5,5) = 500;
  k_joint_gains_(6,6) = 200;
  ROS_INFO_STREAM("K (joint stiffness): " << std::endl <<  k_joint_gains_);

  // Gains for D error damping term
  d_joint_gains_ = Eigen::MatrixXd::Identity(7, 7);
  ROS_INFO_STREAM("D (joint damping): " << std::endl << d_joint_gains_);
  d_joint_gains_(0,0) = 5; d_joint_gains_(1,1) = 5; d_joint_gains_(2,2) = 5;
  d_joint_gains_(3,3) = 2; d_joint_gains_(4,4) = 2; d_joint_gains_(5,5) = 2;
  d_joint_gains_(6,6) = 1;

  // Gains for feed-forward damping term
  d_ff_joint_gains_ = Eigen::MatrixXd::Identity(7, 7);


  return true;
}

void CartesianTwistImpedanceController::starting(const ros::Time& /*time*/) {

  // Get robot current/initial joint state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set desired point to current state
  position_d_           = initial_transform.translation();
  orientation_d_        = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_    = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  if (!q_d_nullspace_initialized_) {
    q_d_nullspace_ = q_initial;
    q_d_nullspace_initialized_ = true;
    ROS_INFO_STREAM("Desired nullspace position (from q_initial): " << std::endl << q_d_nullspace_);
  }
}

void CartesianTwistImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_tool(7);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // This is the if statement that should be made into two different controllers
  if (_goto_home){    
    ROS_INFO_STREAM ("Moving robot to home joint configuration.");            
    
    // Variables to control robot in joint space 
    Eigen::VectorXd q_error(7), dq_desired(7), dq_filtered(7), q_desired(7), q_delta(7);

    // Compute linear DS in joint-space
    q_error = q - q_home_;
    dq_desired = -A_jointDS_home_ * q_error;

    // Filter desired velocity to avoid high accelerations!
    dq_filtered = (1-dq_filter_params_)*dq + dq_filter_params_*dq_desired;

    ROS_INFO_STREAM ("Joint position error:" << q_error.norm());
    // ROS_INFO_STREAM ("dq_desired:" << std::endl << dq_desired);
    // ROS_INFO_STREAM ("dq_filtered:" << std::endl << dq_filtered);

    // Integrate to get desired position
    q_desired = q + dq_desired*dt_;

    // Desired torque: Joint PD control with damping ratio = 1
    tau_task << -k_joint_gains_*(q - q_desired) - d_ff_joint_gains_*dq;

    // Desired torque: Joint PD control
    // tau_task << -0.50*k_joint_gains_ * q_delta - 2.0*d_joint_gains_*(dq - dq_desired) - d_ff_joint_gains_*dq;

    if (q_error.norm() < jointDS_epsilon_){
      ROS_INFO_STREAM ("Finished moving to initial joint configuration. Continuing with desired Cartesian task!" << std::endl);  
      _goto_home = false;
    }    

    // convert to eigen
    Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

    // set desired point for Cartesian impedance controller to current state
    position_d_  = current_transform.translation();

  }
  else{

    // IF NOT GO_HOME -> DO CARTESIAN IMPEDANCE CONTROL
    // ROS_INFO_STREAM ("Doing Cartesian Impedance Control");            
    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;

    // Simple integration of DS
    // ROS_INFO_STREAM("Current ee position: " << position);
    // ROS_INFO_STREAM("Desired velocity from DS: " << velocity_d_);
    // ROS_INFO_STREAM("Desired ee position from DS: " << position_d_);
    error.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    // Cartesian PD control with damping ratio = 1
    tau_task << jacobian.transpose() *(-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  
    // ROS_INFO_STREAM("error: " << error);
    // ROS_INFO_STREAM("Tau task: " << tau_task);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // pseudoinverse for nullspace handling
  // kinematic pseudoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        nullspace_damping_ * dq);

  // Compute tool compensation (scoop/camera in scooping task)
  if (activate_tool_compensation_)
    tau_tool << jacobian.transpose() * tool_compensation_force_;
  else
    tau_tool.setZero();

  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis - tau_tool;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  // cartesian_stiffness_ =
  //     filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  // cartesian_damping_ =
  //     filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  // nullspace_stiffness_ =
  //     filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  
  // // position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  // position_d_ = position_d_target_;
  // orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  cartesian_stiffness_ = cartesian_stiffness_target_;
  cartesian_damping_ = cartesian_damping_target_;
  nullspace_stiffness_ = nullspace_stiffness_target_;
  nullspace_damping_ = nullspace_damping_target_;
  position_d_ = position_d_target_;
  orientation_d_ = orientation_d_target_;
  // ROS_INFO_STREAM("filtered cartesian_stiffness_: " << std::endl <<  cartesian_stiffness_);
}

Eigen::Matrix<double, 7, 1> CartesianTwistImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianTwistImpedanceController::complianceParamCallback(
    franka_interactive_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  // cartesian_stiffness_target_.setIdentity();
  // cartesian_stiffness_target_.topLeftCorner(3, 3)
  //     << config.translational_stiffness * Eigen::Matrix3d::Identity();
  // cartesian_stiffness_target_.bottomRightCorner(3, 3)
  //     << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  // cartesian_damping_target_.setIdentity();
  
  // Damping ratio = 1
  // cartesian_damping_target_.topLeftCorner(3, 3)
  //     << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  // cartesian_damping_target_.bottomRightCorner(3, 3)
  //     << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  
  // nullspace_stiffness_target_ = config.nullspace_stiffness;

  activate_tool_compensation_ = config.activate_tool_compensation;

  // ROS_INFO_STREAM("cartesian_stiffness_target_: " << std::endl <<  cartesian_stiffness_target_);
  // ROS_INFO_STREAM("cartesian_damping_target_: " << std::endl <<  cartesian_damping_target_);
  // ROS_INFO_STREAM("nullspace_stiffness_target_: " << std::endl <<  nullspace_stiffness_target_);
}

void CartesianTwistImpedanceController::desiredCartesianStiffnessCallback(
    const std_msgs::Float64MultiArray& msg) {
  // https://gist.github.com/alexsleat/1372845
  if (msg.data.size() != 6) {
    ROS_ERROR("CartesianTwistImpedanceController: Invalid ROS message for desiredCartesianStiffnessCallback provided");
    throw std::invalid_argument("Aborting controller!");
  }
  
  cartesian_stiffness_target_.setIdentity();
  cartesian_damping_target_.setIdentity();
  for (int i = 0; i < 6; i ++) {
    cartesian_stiffness_target_(i,i) = msg.data[i];
  }
  // Damping ratio = 1
  for (int i = 0; i < 6; i ++) {
    cartesian_damping_target_(i,i) = 2.0 * sqrt(msg.data[i]);
  }
  ROS_WARN_STREAM("[desiredCartesianStiffnessCallback]: cartesian_stiffness_target_: " << std::endl <<  cartesian_stiffness_target_);
  ROS_WARN_STREAM("[desiredCartesianStiffnessCallback]: cartesian_damping_target_: " << std::endl <<  cartesian_damping_target_);
}

void CartesianTwistImpedanceController::desiredNullspaceStiffnessCallback(
    const std_msgs::Float64MultiArray& msg) {
  if (msg.data.size() != 7) {
    ROS_ERROR("CartesianTwistImpedanceController: Invalid ROS message for desiredNullspaceStiffnessCallback provided");
    throw std::invalid_argument("Aborting controller!");
  }
  nullspace_stiffness_target_.setIdentity();
  nullspace_damping_target_.setIdentity();
  for (int i = 0; i < 7; i ++) {
    nullspace_stiffness_target_(i,i) = msg.data[i];
  }
  // Damping ratio = 1
  for (int i = 0; i < 7; i ++) {
    nullspace_damping_target_(i,i) = 2.0 * sqrt(msg.data[i]);
  }
  ROS_WARN_STREAM("[desiredNullspaceStiffnessCallback]: nullspace_stiffness_target_: " << std::endl <<  nullspace_stiffness_target_);
  ROS_WARN_STREAM("[desiredNullspaceStiffnessCallback]: nullspace_damping_target_: " << std::endl <<  nullspace_damping_target_);
}

void CartesianTwistImpedanceController::desiredExternalToolCompensationCallback(
    const std_msgs::Float64MultiArray& msg) {
  if (msg.data.size() != 6) {
    ROS_ERROR("CartesianTwistImpedanceController: Invalid ROS message for desiredExternalToolCompensationCallback provided");
    throw std::invalid_argument("Aborting controller!");
  }
  tool_compensation_force_.setZero();
  for (int i = 0; i < 6; i ++) {
    tool_compensation_force_(i) = msg.data[i];
  }
  ROS_WARN_STREAM("[desiredExternalToolCompensationCallback]: tool_compensation_force_: " << std::endl <<  tool_compensation_force_);
}

void CartesianTwistImpedanceController::desiredTwistCallback(
    const geometry_msgs::TwistConstPtr& msg) {

  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());

  velocity_d_         << msg->linear.x, msg->linear.y, msg->linear.z;
  position_d_target_  << position + velocity_d_*dt_*100;

  // ROS_INFO_STREAM("[CALLBACK] Desired velocity from DS: " << velocity_d_);
  // ROS_INFO_STREAM("[CALLBACK] Desired ee position from DS: " << position_d_target_);


  // position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  
  // Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  // orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
  //     msg->pose.orientation.z, msg->pose.orientation.w;
  
  // if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
  //   orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  // }
}

}  // namespace franka_interactive_controllers

PLUGINLIB_EXPORT_CLASS(franka_interactive_controllers::CartesianTwistImpedanceController,
                       controller_interface::ControllerBase)
