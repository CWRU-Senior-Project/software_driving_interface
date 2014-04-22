/*
 * SDI_state.h
 *
 *  Created on: Mar 21, 2014
 *      Author: democritus5589
 */

#ifndef _SDI_STATE_H_
#define _SDI_STATE_H_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include "software_driving_interface/HDI_feedback.h"
#include <string>

using namespace std;
using namespace software_driving_interface;

namespace software_driving_interface
{
   class SDI_state
   {
      public:
         int run(int argc, char **argv);

      private:
         /*--------------------------------
          *   Properties
          *--------------------------------*/
         bool messageProcessed;
         double wheelAngle;
         double wheelForce;
         double vibration;
         vector<double> atlasPosition;
         vector<double> atlasLinearVelocity;
         vector<double> atlasAngularVelocity;
         HDI_feedback feedbackMsg;

         /*--------------------------------
          *   Functions
          *--------------------------------*/

         // Bind value from Sim-to-SDI message to class property
         void setWheelAngle(const std_msgs::Float64::ConstPtr& msg);

         // Bind value from Sim-to-SDI message to class property
         void setWheelForce(const std_msgs::Float64::ConstPtr& msg);

         // Set vibration value based on key state of vehicle
         // engine/key = on --> vibration = on, else off
         void setVibration(const std_msgs::Int8::ConstPtr& msg);

         // Set velocity value based on atlas's linear and angular velocity
         void setVelocity(const nav_msgs::Odometry::ConstPtr& msg);

         // Set force based on atlas's linear and angular velocity and wheel angle
         void setHandWheelForce();

         // Set output message values
         void fillFeedbackMsg();

         /*--------------------------------
          *   Logging
          *--------------------------------*/

         // Log message value received individually with custom name
         void logMessage(string name, const std_msgs::Int8::ConstPtr& msg);

         // Log message value received individually with custom name
         void logMessage(string name, const std_msgs::Float64::ConstPtr& msg);

         // Log SDI-to-HDI feedback message
         void logMessage(software_driving_interface::HDI_feedback& msg);
         void logMessage(const nav_msgs::Odometry::ConstPtr& msg);


   }; // end of SDI_state class
} // end of namespace



#endif /* _SDI_STATE_H_ */

