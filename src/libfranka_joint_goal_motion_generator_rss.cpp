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
       case 1  :
          std::cout << "Selected q_home as goal" << std::endl;
          q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
          break;
       case 2  :
        std::cout << "Selected q_init_scoop as goal" << std::endl;
          q_goal = {{-0.9933301728190036, 0.2972493461905292, 0.07672433905072819, -1.8928353563985103, 1.2921060452991062, 1.3111778660879454, 0.09839494459331036}};
          break;
       case 3  :
        std::cout << "Selected q_init_scoop as goal" << std::endl;
          q_goal = {{-1.6262530183565473, 0.36835540500440095, 0.7996468301612609, -1.7092709166376214, 0.9194892226190297, 0.8895511734750535, 0.31249669338448877}};
          break;
       case 4  :
        std::cout << "Selected q_init_scoop (on the right of table) as goal" << std::endl;
          q_goal = {{0.12735585180709236, 0.5619404064646938, 0.6805618834882704, -1.6823562078977885, -1.3440559658978612, 0.7525859880270781, 1.5819390151704902}};
          break;
       case 5  :
        std::cout << "Selected q_init_scoop (on the right of table) as goal" << std::endl;
          q_goal = {{-0.3798102209191597, 0.3738950568236193, 0.7679064830235985, -1.6956826430138754, -1.7372545425227859, 1.1540701936678273, 1.2543177286354388}};
          break;
       case 6  :
        std::cout << "Selected q_init_scoop (on the right of table) as goal" << std::endl;
          q_goal = {{0.0444735907446016, 0.021154987762181367, 0.5044643525575336, -1.9534015166522465, -1.362052292667275, 1.0348031652238634, -0.2960306876649459}};
          break;
       // high starting location
       case 7  :
        std::cout << "Selected q_init_scoop (on the right of table) as goal" << std::endl;
          q_goal = {{0.03587195687283549, -0.13952198328888207, 0.3845826635528029, -1.6133267634010875, -1.4906481852001614, 1.111173628756654, -0.8996329480161268}};
          break;
       case 8  :
        std::cout << "Selected q_init_scoop (in the middle of the table) as goal" << std::endl;
          q_goal = {{-0.00021255541978810503, 0.1255734273435194, 0.0012336395456138227, -2.2089848212634764, 0.001337408654865234, 2.3294771154241243, 0.785000418968366}};
          break;
          // -0.00021255541978810503, 0.1255734273435194, 0.0012336395456138227, -2.2089848212634764, 0.001337408654865234, 2.3294771154241243, 0.785000418968366
       case 9  :
        std::cout << "Selected q_init_scoop (on the top right side of the table) as goal" << std::endl;
          q_goal = {{0.8450992030242781, 0.21778273696880843, 0.048599317568435996, -2.07138197806119, -0.035124700197536794, 2.36906767249673, 1.6893649699555515}};
          break;
       case 10:
        std::cout << "Selected tracing (on the top right side of the table) with scooping tool" << std::endl;
          q_goal = {{0.5186169317814342, 0.47327140679336643, 0.5655727155502884, -1.8982900782468024, -1.4883901341760981, 1.2496956815573967, -0.029926588706774452}};
          break;
       case 11:
        std::cout << "Selected tracing (on the top right side of the table) with scooping tool" << std::endl;
          q_goal = {{0.6927077963203601, 0.489265818006338, 0.06339761333551898, -1.8396939527517642, -0.061229715592821206, 2.331856907707782, 1.5879182912551706}};
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
