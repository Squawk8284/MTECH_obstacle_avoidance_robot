// #include <stdio.h>
// #include <stdint.h>
// #include <stdlib.h>
// #include <termios.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <signal.h>
// #include "0xRobotcpplib.h" // Ensure this is in the include path

// #define DEVICE_PORT "/dev/ttyRobot"

// // Global variables
// lib0xRobotCpp robot;
// void *hSerial;

// // Function to reset terminal to default
// struct termios orig_termios;
// void reset_terminal_mode() {
//     tcsetattr(0, TCSANOW, &orig_termios);
// }

// // Function to set terminal to raw mode
// void set_terminal_mode() {
//     struct termios new_termios;

//     // Save original terminal attributes
//     tcgetattr(0, &orig_termios);
//     atexit(reset_terminal_mode);

//     // Set terminal to raw mode
//     new_termios = orig_termios;
//     new_termios.c_lflag &= ~(ICANON | ECHO);
//     tcsetattr(0, TCSANOW, &new_termios);
// }

// // Function to set stdin to non-blocking mode
// void set_nonblocking_mode() {
//     int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
//     fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
// }

// // Signal handler for cleanup
// void sigint_handler(int signum) {
//     (void)signum; // Suppress unused parameter warning
//     robot.stop(hSerial);
//     robot.disconnect_comm(hSerial);
//     printf("\nDisconnected from the robot. Exiting...\n");
//     exit(0);
// }

// int main() {
//     char input = 0;

//     // Set terminal to raw and non-blocking mode
//     set_terminal_mode();
//     set_nonblocking_mode();

//     // Handle Ctrl+C
//     signal(SIGINT, sigint_handler);

//     // Connect to the robot
//     hSerial = robot.connect_comm(DEVICE_PORT);
//     if (hSerial == NULL) {
//         printf("Failed to connect to the robot on %s!\n", DEVICE_PORT);
//         return -1;
//     }
//     printf("Successfully connected to the robot on %s\n", DEVICE_PORT);

//     // Initialize robot settings
//     robot.stop(hSerial);
//     robot.resetMotorEncoderCount(hSerial);
//     robot.setAcceleration(hSerial, 4);
//     robot.setLinearVelocity_meterspersec(hSerial, 0.25);
//     robot.setSafetyTimeout(hSerial, 0);
//     robot.setSafety(hSerial, 0);

//     printf("Control the robot using W/A/S/D keys. Press X to stop and Q to quit.\n");

//     // Main loop
//     while (1) {
//         input = getchar();

//         if (input != EOF) { // Only process valid inputs
//             switch (input) {
//                 case 'w': // Move forward
//                     robot.forward(hSerial);
//                     printf("Moving forward...\n");
//                     break;
//                 case 'a': // Turn left
//                     robot.left(hSerial);
//                     printf("Turning left...\n");
//                     break;
//                 case 's': // Move backward
//                     robot.backward(hSerial);
//                     printf("Moving backward...\n");
//                     break;
//                 case 'd': // Turn right
//                     robot.right(hSerial);
//                     printf("Turning right...\n");
//                     break;
//                 case 'x': // Stop robot
//                     robot.stop(hSerial);
//                     printf("Robot stopped.\n");
//                     break;
//                 case 'q': // Quit program
//                     printf("Quitting...\n");
//                     robot.stop(hSerial);
//                     robot.disconnect_comm(hSerial);
//                     return 0;
//                 default:
//                     robot.stop(hSerial);
//                     printf("Invalid input. Use W/A/S/D to move, X to stop, Q to quit.\n");
//                     break;
//             }
//         }

//         // Small delay to prevent CPU overuse
//         usleep(100000); // 100 ms
//     }

//     return 0;
// }
