
/**
 * @addtogroup atlas
 *@{
 */
#ifndef _ATLAS_MESSAGES_H_
#define _ATLAS_MESSAGES_H_
/** @file atlas_messages.h
* @brief This file specifies the messages for this modules used to transmit xxxx data via ipc to other modules.xxxxxxxxxxxxxxxxx
*/

/** @def ATLAS_RESET_OCCURRED_NAME
* @brief Control message to pass to the process control module. Always capitals finishes with _RESET_OCCURRED_NAME and the value is small letters the same without the suffix name.
*/
#define ATLAS_RESET_OCCURRED_NAME "atlas_reset_occurred"
typedef carmen_default_message atlaso_reset_occurred_message;

/** @def ATLAS_RESET_COMMAND_NAME
 * @brief Control message to pass to the process control module. Always capitals finishes with _RESET_COMMAND_NAME and the value is small letters the same without the suffix name.
 */
#define ATLAS_RESET_COMMAND_NAME "atlas_reset_command"
typedef carmen_default_message atlas_reset_command_message;

/** @struct robot_motion_command_message
* @brief This is the command message, that controls the speed and direction of the atlas robots.
*/
typedef struct {
  double timestamp;
  double brake;			/// values 0-1, 0 without brake 1-full brake
  double dir;			/// radians acc. param_daemon
  double speed;			/// m/s
  int priority;			/// 1-inf. to overrride FIFO
  double life;			/// -1 keep that message being executed forever. 0-inf period that the motion message can be kept alive
  char *host;
} robot_motion_command_message;

/** @def ROBOT_MOTION_COMMAND_NAME
 * @brief Name of the command message for the atlas robots
*/
#define      ROBOT_MOTION_COMMAND_NAME       (char*)"robot_motion_command"

/** @def ROBOT_MOTION_COMMAND_FMT
 * @brief Format of the command message for the atlas robots
*/
#define      ROBOT_MOTION_COMMAND_FMT        (char*)"{double,double,double,double,int,double,string}"

/** @struct robot_lights_command_message
* @brief Lights command message
*/
typedef struct {
	double timestamp;
	int turnright;		/**< DDD */
	int turnleft;		/**< DDD */
	int headlights;		/**< DDD */
	int taillights;		/**< DDD */
	int reverselights;	/**< DDD */
	char *host;
} robot_lights_command_message;

/** @def ROBOT_LIGHTS_COMMAND_NAME
 * @brief Name of the lights command message
*/
#define      ROBOT_LIGHTS_COMMAND_NAME       (char*)"robot_lights_command"

/** @def ROBOT_LIGHTS_COMMAND_FMT
 * @brief Format of the lights command message
*/
#define      ROBOT_LIGHTS_COMMAND_FMT        (char*)"{double,int,int,int,int,int,string}"

/*! @struct robot_vert_sign_command_message
* @brief robot_vert_sign_command_message struct where is defined all 6 different signs.

*/
typedef struct
{
	double timestamp;
	int vert_sign; /** can be 0-6*/
	char* host;
} robot_vert_sign_command_message;

/** @def ROBOT_VERT_SIGN_COMMAND_NAME
 * @brief Name of the vertical sign command
*/
#define      ROBOT_VERT_SIGN_COMMAND_NAME       (char*)"robot_vert_sign_command"

/** @def ROBOT_VERT_SIGN_COMMAND_FMT
 * @brief Format of the vertical sign command
*/
#define      ROBOT_VERT_SIGN_COMMAND_FMT        (char*)"{double,int,string}"

/** @struct robot_status_message
* @brief Base status message for the atlas robots
*/
typedef struct {
	double timestamp;
	double brake;		///0- not braking, 1-full brake
	double dir;			///radians
	double speed;		///m/s
	double x;			///absolute X position (m)
	double y;			///absolute Y position (m)
	double orientation; ///absolute Orientation (rad)
	double distance_traveled;	///Total distance traveled (m)
	int turnright;		///bool
	int turnleft;		///bool
	int headlights;		///bool
	int taillights;		///bool
	int reverselights;	///bool
	int cross_sensor;	///bool
	int vert_sign;		///enum
	int errors;			///enum @ atlas_interface.h
	char *host;
} robot_status_message;

/** @def ROBOT_STATUS_NAME
 * @brief Name of the status message
*/
#define      ROBOT_STATUS_NAME       (char*)"robot_status"

/** @def ROBOT_STATUS_FMT
 * @brief Format of the status message
*/
#define      ROBOT_STATUS_FMT        (char*)"{double,double,double,double,double,double,double,double,int,int,int,int,int,int,int,int,string}"

/** @struct robot_brake_toggle_message
* @brief
*/
typedef struct {
	double timestamp;
	int toggle;
	char *host;
} robot_brake_toggle_message;

/** @def ROBOT_STATUS_NAME
 * @brief Name of the status message
*/
#define      ROBOT_BRAKE_TOGGLE_NAME       (char*)"robot_brake_toggle_status"

/** @def ROBOT_STATUS_FMT
 * @brief Format of the status message
*/
#define      ROBOT_BRAKE_TOGGLE_FMT        (char*)"{double,int,string}"


#endif
/**
*@}
*/

