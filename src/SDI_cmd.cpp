#include "software_driving_interface/SDI_cmd.h"
#include <sstream>
#include <string>

using namespace software_driving_interface;
using namespace std;

SDI_cmd::SDI_cmd()
{
   messageProcessed = false;
}

void SDI_cmd::extractMsgValues(const software_driving_interface::HDI_control::ConstPtr& msg)
{
   SDI_cmd::logMessage(msg);

   // update values from HDI message
   this->gasPedalPercentMsg.data = msg->gas_pos;
   this->brakePedalPercentMsg.data = msg->brake_pos;
   this->wheelAngleMsg.data = msg->wheel_angle;
   this->handBrakePercentMsg.data = 0;
   this->directionValueMsg.data = getSimGearFromHDIGear(msg->gear);
   this->keyValueMsg.data = 1;
   this->vibrationMsg.data = msg->vibration;

   validateMsgInput();
}

int SDI_cmd::getSimGearFromHDIGear(int gear)
{
   int outputGear = 0;

   switch (gear)
   {
      // Park
      case 0:
         outputGear = 2;
         break;

      // Reverse
      case 1:
         outputGear = -1;
         break;

      // Neutral
      case 2:
         outputGear = 0;
         break;

      // Drive 1
      case 3:
         outputGear = 1;
         break;

      // Drive 2
      case 4:
         outputGear = 1;
         break;

      default:
         ROS_WARN_NAMED("Testing_WARN", "Invalid gear value received. Default to neutral.");
         break;
   }

   return outputGear;
}

void SDI_cmd::logMessage(const software_driving_interface::HDI_control::ConstPtr& msg)
{
   stringstream ss;
   ss << "HDI message received by SDI and logged. Message contents:\n";
   ss << "wheel_angle:\t" << msg->wheel_angle << "\n";
   ss << "gas_pos:\t" << msg->gas_pos << "\n";
   ss << "brake_pos:\t" << msg->brake_pos << "\n";
   ss << "gear:\t" << msg->gear << "\n";
   ss << "vibration:\t" << msg->vibration << "\n";

   ROS_WARN_NAMED("Testing_WARN", "%s", ss.str().c_str());
}

void SDI_cmd::validateMsgInput()
{
   // Restrict gas position values
   if (MIN_GAS_PERCENT > this->gasPedalPercentMsg.data)
   {
      ROS_WARN("Value of gas position received from HDI (%f) is LESS than the minimum acceptable value (%f). Value set to min.", gasPedalPercentMsg.data, MIN_GAS_PERCENT);
      gasPedalPercentMsg.data = MIN_GAS_PERCENT;
   }
   else if (MAX_GAS_PERCENT < this->gasPedalPercentMsg.data)
   {
      ROS_WARN("Value of gas position received from HDI (%f) is GREATER than the maximum acceptable value (%f). Value set to max.", gasPedalPercentMsg.data, MAX_GAS_PERCENT);
      gasPedalPercentMsg.data = MAX_GAS_PERCENT;
   }

   // Restrict brake pedal position values
   if (MIN_BRAKE_PEDAL_PERCENT > this->brakePedalPercentMsg.data)
   {
      ROS_WARN("Value of brake position received from HDI (%f) is LESS than the minimum acceptable value (%f). Value set to min.", brakePedalPercentMsg.data, MIN_BRAKE_PEDAL_PERCENT);
      brakePedalPercentMsg.data = MIN_BRAKE_PEDAL_PERCENT;
   }
   else if (MAX_BRAKE_PEDAL_PERCENT < this->brakePedalPercentMsg.data)
   {
      ROS_WARN("Value of brake position received from HDI (%f) is GREATER than the maximum acceptable value (%f). Value set to max.", brakePedalPercentMsg.data, MAX_BRAKE_PEDAL_PERCENT);
      brakePedalPercentMsg.data = MAX_BRAKE_PEDAL_PERCENT;
   }
/*
   // Restrict wheel position
   if (MIN_WHEEL_POS() > this->wheelAngleMsg.data)
   {
      ROS_WARN("Value of wheel angle received from HDI (%f) is LESS than the minimum acceptable value (%f). Value set to min.", wheelAngleMsg.data, MIN_WHEEL_POS());
      wheelAngleMsg.data = MIN_WHEEL_POS();
   }
   else if (MAX_WHEEL_POS() < this->wheelAngleMsg.data)
   {
      ROS_WARN("Value of wheel angle received from HDI (%f) is GREATER than the maximum acceptable value (%f). Value set to max.", wheelAngleMsg.data, MAX_WHEEL_POS());
      wheelAngleMsg.data = MAX_WHEEL_POS();
   }
*/

   // Restrict vehicle direction
   if ((-1 > directionValueMsg.data) || (2 < directionValueMsg.data))
   {
      ROS_WARN("Value of vehicle direction received from HDI (%d) is not VALID. Acceptable values are (1: REVERSE, 2: NEUTRAL, 3/4: FORWARD, 0: PARK). Value set to NEUTRAL.", directionValueMsg.data);
      directionValueMsg.data = 1;
   }

   if (2 == directionValueMsg.data)
   {
      handBrakePercentMsg.data = 1.0;
      directionValueMsg.data = 0.0;
   }
   else
   {
      handBrakePercentMsg.data = 0.0;
   }

   // Restrict vibration percent
   if (MIN_VIBRATION_PERCENT > this->vibrationMsg.data)
   {
      ROS_WARN("Value of vibration received from HDI (%f) is LESS than the minimum acceptable value (%f). Value set to min.", vibrationMsg.data, MIN_VIBRATION_PERCENT);
      vibrationMsg.data = MIN_VIBRATION_PERCENT;
   }
   else if (MAX_VIBRATION_PERCENT < this->vibrationMsg.data)
   {
      ROS_WARN("Value of vibration received from HDI (%f) is GREATER than the maximum acceptable value (%f). Value set to max.", vibrationMsg.data, MAX_VIBRATION_PERCENT);
      vibrationMsg.data = MAX_VIBRATION_PERCENT;
   }

   messageProcessed = true;
}

int SDI_cmd::run(int argc, char **argv)
{
   ros::init(argc, argv, "SDI_input");
   ros::NodeHandle handle;

   // Instantiate Subscribers
   ros::Subscriber subHDICmd = handle.subscribe("HDI/cmd", 1000, &SDI_cmd::extractMsgValues, this);

   // Instantiate Publishers
   ros::Publisher pubKeyCmd = handle.advertise<std_msgs::Int8>("drc_vehicle_xp900/key/cmd", 1000);
   ros::Publisher pubHandBrakeCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/hand_brake/cmd", 1000);
   ros::Publisher pubGasPedalCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/gas_pedal/cmd", 1000);
   ros::Publisher pubHandWheelCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/hand_wheel/cmd", 1000);
   ros::Publisher pubBrakePedalCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/brake_pedal/cmd", 1000);
   ros::Publisher pubVehicleDirectionCmd = handle.advertise<std_msgs::Int8>("drc_vehicle_xp900/direction/cmd", 1000);

   ros::Rate loop_rate(500); // 100 Hz
   ros::spinOnce();

   while (ros::ok())
   {
      if (true)//messageProcessed)
      {
         messageProcessed = false;
         logMessage();

         //	if (key = on and vehicle is in neutral) or (key is off), publish
         if ((0 == keyValueMsg.data) || ((1 == keyValueMsg.data) && (0 == directionValueMsg.data)))
         {
            pubKeyCmd.publish(keyValueMsg);
         }

         // Publish SDI-to-Sim msgs
         pubHandBrakeCmd.publish(handBrakePercentMsg);
         pubGasPedalCmd.publish(gasPedalPercentMsg);
         pubHandWheelCmd.publish(wheelAngleMsg);
         pubBrakePedalCmd.publish(brakePedalPercentMsg);
         pubVehicleDirectionCmd.publish(directionValueMsg);
      }

      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}

void SDI_cmd::logMessage()
{
   SDI_cmd& controller = *this;
	
   stringstream ss;
   ss << "SDI messages sent to simulator and logged. Message contents:\n";
   ss << "Gas Pedal %:\t" << controller.gasPedalPercentMsg.data << "\n";
   ss << "Brake Pedal %:\t" << controller.brakePedalPercentMsg.data << "\n";
   ss << "Steering Wheel Angle (rad):\t" << controller.wheelAngleMsg.data << "\n";
   ss << "Hand Brake %:\t" << controller.handBrakePercentMsg.data << "\n";
   ss << "Direction Value:\t" << controller.directionValueMsg.data << "\n";
   ss << "Key Value:\t" << controller.keyValueMsg.data;

   // Avoid resending key commands unless vehicle is off
   if (!((0 == keyValueMsg.data) || ((1 == keyValueMsg.data) && (0 == directionValueMsg.data))))
   {
      ss << "\t(Not sent to sim)";
   }

   ss << "\n";

   ROS_WARN_NAMED("Testing_WARN", "%s", ss.str().c_str());
}

int main(int argc, char **argv)
{
   SDI_cmd cmd;
   cmd.run(argc, argv);
}
