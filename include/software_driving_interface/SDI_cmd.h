/*
 * SDI_cmd.h
 *
 *  Created on: Mar 21, 2014
 *      Author: democritus5589
 */

#ifndef _SDI_CMD_H_
#define _SDI_CMD_H_

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "software_driving_interface/HDI_control.h"
#include <string>
using namespace std;
using namespace software_driving_interface;

namespace software_driving_interface
{
   static const double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;

   class SDI_cmd
   {
      public:
         SDI_cmd();
         int run(int argc, char **argv);

      private:
         /*--------------------------------
          *   Constants
          *--------------------------------*/

         // Specify range of acceptable gas pedal values
         static const double MIN_GAS_PERCENT = 0.0;
         static const double MAX_GAS_PERCENT = 1.0;

         // Specify range of acceptable brake pedal values
         static const double MIN_BRAKE_PEDAL_PERCENT = 0.0;
         static const double MAX_BRAKE_PEDAL_PERCENT = 0.0;

         // Specify acceptable direction values
         static const int VALID_DIRECTION_VALUES[4];// = { 0, 1, 2, 3 };

         // Specify range of acceptable hand brake values
         static const double MIN_HAND_BRAKE_PERCENT = 0.0;
         static const double MAX_HAND_BRAKE_PERCENT = 1.0;

         // Specify range of acceptable wheel position values
         static double MIN_WHEEL_POS() { return -5 * PI; };
         static double MAX_WHEEL_POS() { return 5 * PI; };

         // Specify acceptable ignition key values
         static const int VALID_KEY_VALUES[2];// = { 0, 1 };

         // Specify range of acceptable vibration values
         static const double MIN_VIBRATION_PERCENT = 0.0;
         static const double MAX_VIBRATION_PERCENT = 1.0;

         /*--------------------------------
          *   Properties
          *--------------------------------*/
         bool messageProcessed;
         std_msgs::Float64 gasPedalPercentMsg;
         std_msgs::Float64 brakePedalPercentMsg;
         std_msgs::Float64 wheelAngleMsg;
         std_msgs::Float64 handBrakePercentMsg;
         std_msgs::Int8 directionValueMsg;
         std_msgs::Int8 keyValueMsg;
         std_msgs::Float64 vibrationMsg;

         /*--------------------------------
          *   Functions
          *--------------------------------*/
         // Bind values from HDI-to-SDI control messages to class properties
         void extractMsgValues(const software_driving_interface::HDI_control::ConstPtr& msg);

         // Validate and restrict SDI-to-Sim message values
         void validateMsgInput();

         /*--------------------------------
          *   Logging
          *--------------------------------*/

         // Log HDI-to-SDI control message
         void logMessage(const software_driving_interface::HDI_control::ConstPtr& msg);

         // Log SDI-to-Sim control messages
         void logMessage();

   }; // end of SDI_cmd class

   // Specify acceptable values for direction and key state
   const int SDI_cmd::VALID_DIRECTION_VALUES[4] = { 0, 1, 2, 3 };
   const int SDI_cmd::VALID_KEY_VALUES[2] = { 0, 1 };

} // end of namespace



#endif /* _SDI_CMD_H_ */

