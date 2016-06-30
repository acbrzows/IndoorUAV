/****************************************************************************
 *
 * Copyright (C) 2014 Flavio Fontana & Luis Rodrigues. All rights reserved.
 * Author: Flavio Fontana <fly.fontana@gmail.com>
 * Author: Luis Rodrigues <luis.rodrigues@terabee.com>

 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name TerarangerOne nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <string>

#include "terarangerone/terarangerone.h"

// double range_values[8]={0.0};

std::vector<double> range_values(8,0);

namespace terarangerone
{

TerarangerOne::TerarangerOne()
{
  // Get paramters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_, std::string("/dev/ttyUSB0"));

  // Publishers
  range_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>("terarangerone", 1);

  // Create serial port
  serial_port_ = new SerialPort();

  // Set callback function for the serial ports
  serial_data_callback_function_ = boost::bind(&TerarangerOne::serialDataCallback, this, _1);
  serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);

  // Connect serial port
  if (!serial_port_->connect(portname_))
  {
    ros::shutdown();
    return;
  }

  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());

  // Set operation Mode
  setMode(BINARY_MODE);

  // Dynamic reconfigure
  dyn_param_server_callback_function_ = boost::bind(&TerarangerOne::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);
}

TerarangerOne::~TerarangerOne()
{
}

uint8_t TerarangerOne::crc8(uint8_t *p, uint8_t len)
{
  uint16_t i;
  uint16_t crc = 0x0;

  while (len--)
  {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

void TerarangerOne::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  std_msgs::Float64MultiArray range_msg;
  //range_msg.layout = 0.0593;
  //range_msg.data = 14.0;

  if (single_character != 'T' && buffer_ctr < 4)
  {
    // not begin of serial feed so add char to buffer
    input_buffer[buffer_ctr++] = single_character;
    return;
  }
  else if (single_character == 'T')
  {
    if (buffer_ctr == 4)
    {
      // end of feed, calculate
      int16_t crc = crc8(input_buffer, 3);

      if (crc == input_buffer[3])
      {
        int16_t range = input_buffer[1] << 8;
        range |= input_buffer[2];

        if (range < 14000 && range > 200)
        {
          // range_msg.header.stamp = ros::Time::now();
          // range_msg.header.seq = seq_ctr++;
          // range_msg.data[0] = range * 0.001; // convert to m
          range_values[0] = range *0.001;
	  range_msg.data = range_values;
          range_publisher_.publish(range_msg);
        }
        ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range_msg.data[0]);
      }
      else
      {
        ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
      }
    }
    else
    {
      ROS_DEBUG("[%s] reveived T but did not expect it, reset buffer without evaluating data",
               ros::this_node::getName().c_str());
    }
  }
  else
  {
    ROS_DEBUG("[%s] buffer_overflowed without receiving T, reset input_buffer", ros::this_node::getName().c_str());
  }
  // reset
  buffer_ctr = 0;

  // clear struct
  bzero(&input_buffer, BUFFER_SIZE);

  // store T
  input_buffer[buffer_ctr++] = 'T';
}

void TerarangerOne::setMode(char c)
{
  serial_port_->sendChar(c);
}

void TerarangerOne::dynParamCallback(const terarangerone::TerarangerOneConfig &config, uint32_t level)
{
  if (config.Mode == terarangerone::TerarangerOne_Fast)
  {
    setMode(FAST_MODE);
  }

  if (config.Mode == terarangerone::TerarangerOne_Precise)
  {
    setMode(PRECISE_MODE);
  }
 
  if (config.Mode == terarangerone::TerarangerOne_Outdoor)
  {
    setMode(OUTDOOR_MODE);
  }
}

} // namespace terarangerone

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terarangerone");
  terarangerone::TerarangerOne tera_bee;
  ros::spin();

  return 0;
}
