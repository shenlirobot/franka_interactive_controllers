// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Easy non-franka_ros executable to send robot to desired joint configurations. 
// Caution: It won't work when franka_ros/franka_control is running!

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

// #include <franka_motion_generators/libfranka_joint_motion_generator.h>
#include <libfranka_joint_motion_generator.h>

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

int main(int argc, char** argv) {

  std::string franka_ip = "172.16.0.2";

  // Check whether the required arguments were passed replace this with rosparam!
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <goal_id>" << std::endl;
    return -1;
  }

  try {
    
    // Connect to robot.
    franka::Robot robot(franka_ip);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});


    // First move the robot to a suitable joint configuration
    int goal_id = std::stod(argv[1]);

    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    switch(goal_id) {
       case 0: // tracing experiment config
        std::cout << "home configuration for table-top manipulation" << std::endl;
          q_goal = {{0.0001542171229130441, -0.7873074731652728, -0.006526418591684004, -2.357169394455308, -0.0005176712596116381, 1.5713411465220979, 0.7850599268091134}};
          break;            
       case 1: // kitchen config such that gripper is visible in the scene camera, used for old data collection 
        std::cout << "home configuration for kitchen env" << std::endl;
          q_goal = {{0.03989923506243186, -0.8795352630680547, 0.02790805097202798, -2.131082794189453, -0.10203364571597015, 2.131498757091475, 0.9211458707067053}};
          break;          
       case 2: // kitchen config where arm is more sit back and clock is visible in wrist camera
        std::cout << "second home configuration for kitchen env" << std::endl;
          q_goal = {{0.03888077302278917, -1.448513279697351, 0.008016000580797072, -2.167268103191881, -0.05755834689736334, 1.8755393341781141, 0.8160920021941831}};
          break;               
       case 3: // kitchen config used for new data collection
        std::cout << "third home configuration for kitchen env" << std::endl;
          q_goal = {{0.04128145976907175, -1.0386612259202992, 0.001417798253621213, -1.8167583349076724, -0.058470077317928575, 1.740173071914249, 0.815775183826086}};
          break;
       case 4: // kitchen config where arm is high up looking down, hitting joint limits
        std::cout << "fourth home configuration for kitchen env" << std::endl;
          q_goal = {{-0.09227837615444125, -0.5005236509501817, -0.016475427751266442, -1.031010590660899, 0.04656340210636457, 1.3398803110168467, 0.719558948463621}};
          break;
    }

    MotionGenerator motion_generator(0.6, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }

  return 0;
}
