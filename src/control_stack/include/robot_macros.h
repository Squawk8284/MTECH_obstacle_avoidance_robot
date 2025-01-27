#ifndef __ROBOT_MACROS__
#define __ROBOT_MACROS__

#define ROBOT_PORT "/dev/ttyRobot" // Run the script for udev beforehand
#define IMU_PORT "/dev/ttyIMU" // Run the script for udev beforehand


#define ROB_COM_HANDLE    RobotInt->comm_handle
#define IMU_COM_HANDLE    IMUInt->comm_handle


// ***********************************************************
//         ROBOT MACROS AND VARIABLES FOR DIMENSIONS
// ***********************************************************

#define LENGTH_IN_MM (934)
#define BREATH_IN_MM (670)
#define HEIGHT_IN_MM (909)
#define WHEEL_DIAMTER_IN_MM (260)
#define AXEL_LENGTH_IN_MM (590)
#define COUNTS_PER_REVOLUTION (3840)


// ***********************************************************
//         ROBOT MACROS AND VARIABLES FOR TRANSFORMATIONS
// ***********************************************************


#endif //__ROBOT_MACROS__