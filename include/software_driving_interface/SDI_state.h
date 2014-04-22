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
         void setWheelAngle(const std_msgs::Float64::ConstPtr& msg);
         void setWheelForce(const std_msgs::Float64::ConstPtr& msg);
         void setVibration(const std_msgs::Int8::ConstPtr& msg);
         void setVelocity(const nav_msgs::Odometry::ConstPtr& msg);
         void setHandWheelForce();
         void fillFeedbackMsg();

         /*--------------------------------
          *   Logging
          *--------------------------------*/
         void logMessage(string name, const std_msgs::Int8::ConstPtr& msg);
         void logMessage(string name, const std_msgs::Float64::ConstPtr& msg);
         void logMessage(software_driving_interface::HDI_feedback& msg);
         void logMessage(const nav_msgs::Odometry::ConstPtr& msg);


   }; // end of SDI_state class
} // end of namespace



#endif /* _SDI_STATE_H_ */

