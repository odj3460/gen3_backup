#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <mutex>


#include "ros/ros.h"
#include "rft_sensor_serial/rft_operation.h"
#include "rft_sensor_serial_2/rft_operation_2.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rft_operation_client");


  ros::NodeHandle n;
  ros::NodeHandle nk; 

  ros::ServiceClient client = n.serviceClient<rft_sensor_serial::rft_operation>("rft_serial_op_service");
  ros::ServiceClient client_2 = nk.serviceClient<rft_sensor_serial_2::rft_operation_2>("rft_serial_op_service_2");
  rft_sensor_serial::rft_operation srv;
  rft_sensor_serial_2::rft_operation_2 srv2;

  bool isGo = true;
  bool knownCommand = false;

  while( isGo )
	{
		char cmd = getchar();
		
		if( cmd == 0x1B ) // esc
			isGo = false;
    else if( cmd == 'O' || cmd == 'o' ) // measure Once
		{	srv.request.opType = 10;
      srv.request.param1 = 1;
      srv.request.param2 = 0;
      srv.request.param3 = 0;
      srv2.request.opType = 10;
      srv2.request.param1 = 1;
      srv2.request.param2 = 0;
      srv2.request.param3 = 0;
      knownCommand = true;
    }
		else if( cmd == 'M' || cmd == 'm' ) // measure
		{	srv.request.opType = 11;
      srv.request.param1 = 1;
      srv.request.param2 = 0;
      srv.request.param3 = 0;
      srv2.request.opType = 11;
      srv2.request.param1 = 1;
      srv2.request.param2 = 0;
      srv2.request.param3 = 0;
      knownCommand = true;
    }
		else if( cmd == 'S' || cmd == 's' ) // stop
		{	srv.request.opType = 12;
      srv.request.param1 = 1;
      srv.request.param2 = 0;
      srv.request.param3 = 0;
      srv2.request.opType = 12;
      srv2.request.param1 = 1;
      srv2.request.param2 = 0;
      srv2.request.param3 = 0;
      knownCommand = true;
    }
		else if( cmd == 'B' || cmd == 'b' ) // Set bias
		{	srv.request.opType = 17;
      srv.request.param1 = 1;
      srv.request.param2 = 0;
      srv.request.param3 = 0;
      srv2.request.opType = 17;
      srv2.request.param1 = 1;
      srv2.request.param2 = 0;
      srv2.request.param3 = 0;
      knownCommand = true;
    }
 		else if( cmd == 'U' || cmd == 'u' ) // Set Un-bias
		{	srv.request.opType = 17;
      srv.request.param1 = 0;
      srv.request.param2 = 0;
      srv.request.param3 = 0; 
      srv2.request.opType = 17;
      srv2.request.param1 = 0;
      srv2.request.param2 = 0;
      srv2.request.param3 = 0; 
      knownCommand = true;
    }    
		else if( cmd == 'F' || cmd == 'f' ) // Filter
		{	srv.request.opType = 8;
      srv.request.param1 = 1;
      srv.request.param2 = 10;
      srv.request.param3 = 0;
      srv2.request.opType = 8;
      srv2.request.param1 = 1;
      srv2.request.param2 = 10;
      srv2.request.param3 = 0;
      knownCommand = true;
    }
		else if( cmd == 'G' || cmd == 'g' ) // Filter Go
		{	srv.request.opType = 9;
      srv.request.param1 = 1;
      srv.request.param2 = 1;
      srv.request.param3 = 0;  
      srv2.request.opType = 9;
      srv2.request.param1 = 1;
      srv2.request.param2 = 1;
      srv2.request.param3 = 0;      
      knownCommand = true;
    }
		else
		{
			if( cmd != '\r' && cmd != '\n' )
				printf("UNKNOWN COMMAND\n");
		}


    if(knownCommand)
    {
      if ( client.call(srv) && client_2.call(srv2) )
      {
        ROS_INFO("Service call succeeded, result: %d", srv.response.result);
        ROS_INFO("Service2 call succeeded, result: %d", srv2.response.result);
      }
      else if ( !client.call(srv) && client_2.call(srv2) )
      {
        ROS_ERROR("Failed to call service rft_serial_op_service1");
        return 1;
      }
      else if ( client.call(srv) && !client_2.call(srv2) )
      {
        ROS_ERROR("Failed to call service rft_serial_op_service_2");
        return 1;
      }
      else
      {
        ROS_ERROR("Failed to call all service rft_serial_op_service");
        return 1;
      }

    }

	}




  return 0;

  
}