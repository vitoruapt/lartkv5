#include <Motor.h>

#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>

#include <util.h>
#include <Vector.h>
//#include <SevenSegmentDisplay.h>
#include <Motor.h>
#include <SPI.h> 

// Nominal value of the x, for point 1:       
#define X_NOMINAL_1 681
// Nominal value of the x, for point 2:
#define X_NOMINAL_2 558
// Nominal value of the x, for point 3:
#define X_NOMINAL_3 440
// Nominal value of the y, for point 1:
#define Y_NOMINAL_1 700
// Nominal value of the y, for point 4:
#define Y_NOMINAL_4 600
// Nominal value of the y, for point 7:
#define Y_NOMINAL_7 480

#define ERR_MAX 50
// Definition of the PWM waves (duty cycles):
// The system reset PWM value for Motor 1:
#define PWM_RESET_MOTOR_1 50
// The system reset PWM value for Motor 2:
#define PWM_RESET_MOTOR_2 50
// Minimum PWM value for Motor 1:
#define PWM_MIN_MOTOR_1 50
// Maximum PWM value for Motor 1:
#define PWM_MAX_MOTOR_1 150
// Minimum PWM value for Motor 2:
#define PWM_MIN_MOTOR_2 50
// Maximum PWM value for Motor 2:
#define PWM_MAX_MOTOR_2 100
// The Ethernet Shield MAC ADDRESS:
byte mac_address[]={0x90,0xa2,0xda,0x00,0xf1,0x6d};
// The Ethernet Shield IP ADDRESS:
byte ip[]={10,0,0,213};
//byte ip[]={192,168,1,123};
// Creates the new Ethernet Server:
EthernetServer server(120);
//                           | X nominal |      X minimum    |      X maximum    | Y nominal |     Y minimum     |     Y maximum      |
int resistance_matrix[9][6]={{X_NOMINAL_1,X_NOMINAL_1-ERR_MAX,X_NOMINAL_1+ERR_MAX,Y_NOMINAL_1,Y_NOMINAL_1-ERR_MAX,Y_NOMINAL_1+ERR_MAX},
			                       {X_NOMINAL_2,X_NOMINAL_2-ERR_MAX,X_NOMINAL_2+ERR_MAX,Y_NOMINAL_1,Y_NOMINAL_1-ERR_MAX,Y_NOMINAL_1+ERR_MAX},
			                       {X_NOMINAL_3,X_NOMINAL_3-ERR_MAX,X_NOMINAL_3+ERR_MAX,Y_NOMINAL_1,Y_NOMINAL_1-ERR_MAX,Y_NOMINAL_1+ERR_MAX},
			                       {X_NOMINAL_1,X_NOMINAL_1-ERR_MAX,X_NOMINAL_1+ERR_MAX,Y_NOMINAL_4,Y_NOMINAL_4-ERR_MAX,Y_NOMINAL_4+ERR_MAX},
			                       {X_NOMINAL_2,X_NOMINAL_2-ERR_MAX,X_NOMINAL_2+ERR_MAX,Y_NOMINAL_4,Y_NOMINAL_4-ERR_MAX,Y_NOMINAL_4+ERR_MAX},
			                       {X_NOMINAL_3,X_NOMINAL_3-ERR_MAX,X_NOMINAL_3+ERR_MAX,Y_NOMINAL_4,Y_NOMINAL_4-ERR_MAX,Y_NOMINAL_4+ERR_MAX},
			                       {X_NOMINAL_1,X_NOMINAL_1-ERR_MAX,X_NOMINAL_1+ERR_MAX,Y_NOMINAL_7,Y_NOMINAL_7-ERR_MAX,Y_NOMINAL_7+ERR_MAX},
			                       {X_NOMINAL_2,X_NOMINAL_2-ERR_MAX,X_NOMINAL_2+ERR_MAX,Y_NOMINAL_7,Y_NOMINAL_7-ERR_MAX,Y_NOMINAL_7+ERR_MAX},
			                       {X_NOMINAL_3,X_NOMINAL_3-ERR_MAX,X_NOMINAL_3+ERR_MAX,Y_NOMINAL_7,Y_NOMINAL_7-ERR_MAX,Y_NOMINAL_7+ERR_MAX}};
// Byte for received Data:
int incoming_gear=0;
// Variable to store the gear received from the PC:
int received_gear=0;
// Variable used to store the current gear:
int current_gear=0;
// Pins used to drive the 7-segment display:
//int sevenSeg_pins[3]={A5,A3,A4};
// Boolean variable used to check if the system needs to shift a gear:
byte need_change=0;
// Class of motor 1, which controls the Y direction:
Motor motor1(0);
// Class of motor 2, which controls the X direction:
Motor motor2(1);
// Variable to store the current point of the gear lever:
int current_point;
// Variable used to store the reading of the DOWN Button:
byte reading_button_DN=LOW;
// Variable used to store the reading of the UP Button:
byte reading_button_UP=LOW;
// Variable used to store the reading of the man/auto MODE Button:
byte reading_button_MODE=LOW;
// The running mode of the gearbox. The 0 represents the Automatic Mode, and the 1 represents the Manual Mode.
byte running_mode=1;
void setup()
{
  Serial.begin(9600);
  // Encoder pins:
  pinMode(2,OUTPUT); // Encoder Bit A0;
  pinMode(3,OUTPUT); // Encoder Bit A1;
  pinMode(7,OUTPUT); // Encoder ENABLE;
  // MAN/AUTO Pins:
  pinMode(A2,INPUT); // Yellow (Gear Up) -> Pin A2:
  pinMode(9,INPUT); // Green (Man/Auto Selector) -> Pin 9:
  pinMode(8,INPUT); // Blue (Gear Down) -> Pin 8:
  // Security
  pinMode(A5, INPUT); // safety clutch
  // Analogue pins used as digital pins to activate the 7-segment display:
//  pinMode(A5,OUTPUT); // pin A
//  pinMode(A3,OUTPUT); // pin B
//  pinMode(A4,OUTPUT); // pin C
  // Starting the Ethernet connection:
  Ethernet.begin(mac_address,ip);
  // Starting the Ethernet server:
  server.begin();
  // Both motors are stopped:
  motor1.turn_off();
  // Impose Motor 1 a zero duty cycle:
  motor1.impose_PWM(0);
  // Impose Motor 2 a zero duty cycle:
  motor2.impose_PWM(0);
  // Get the current x resistance value:
  int x_res=analogRead(A1);
  // Get the current y resistance value:
  int y_res=analogRead(A0);
  /*
  ######################################
  This is the standard reset cycle. This cycle runs each time the Arduino is reset, and the sequence of actions is the following:
  1 -> The gear lever is pulled vertically to the neutral position;
  2 -> The gear lever is pulled horizontally to the middle position.
  ######################################
  */
  // First, if the y resistance value is greater than the Neutral's resistance value:
  if(y_res<Y_NOMINAL_4)
  {
    // Motor 1 is activated:
    motor1.turn_on_counterclockwise();
    
    // Motor 1 is slowly moved until it reaches the neutral position:
    while(analogRead(A0)<Y_NOMINAL_4)
    {
//      Serial.print("Emergencia:");
//      Serial.println(digitalRead(A5));
//      Serial.print("A0_y>:");
//      Serial.println(analogRead(A0));

      // Impose a low duty cycle PWM to motor 1:
      motor1.impose_PWM(PWM_RESET_MOTOR_1);
    }
    // Delay to turn off the motor:
    delay(50);
    // Both motors are stopped:
    motor1.turn_off();
  }
  // Or if the y resistance value is lower than the Neutral's resistance value:
  else if (y_res>Y_NOMINAL_4)
  {
    // Motor 1 is activated:
    motor1.turn_on_clockwise();
    // Motor 1 is slowly moved until it reaches the neutral position:
    while(analogRead(A0)>Y_NOMINAL_4)
    {
//      Serial.print("Emergencia:");
//      Serial.println(digitalRead(A5));
//      Serial.print("A1_y<:");      
//      Serial.println(analogRead(A0));

      // Impose a low duty cycle PWM to motor 1:
      motor1.impose_PWM(PWM_RESET_MOTOR_1);
    }
    // Delay to turn off the motor:
    delay(50);
    // Both motors are stopped:
    motor1.turn_off();
  }  
  // Second, if the x resistance value is greater than the Neutral's point 5 resistance value:
  if(x_res<X_NOMINAL_2)
  {
    // Motor 2 is activated:
    motor2.turn_on_clockwise();
    // Motor 2 is slowly moved until it reaches the neutral's point 5 position:
    while(analogRead(A1)<X_NOMINAL_2)
    {
//      Serial.print("A2_x>:");      
//      Serial.println(analogRead(A1));

      // Impose a low duty cycle PWM to motor 2:
      motor2.impose_PWM(PWM_RESET_MOTOR_2);
    }
    // Delay to turn off the motor:
    delay(50);
    // Both motors are stopped:
    motor2.turn_off();
  }
  else if (x_res>X_NOMINAL_2)
  {
    // Motor 2 is activated:
    motor2.turn_on_counterclockwise();
    // Motor 2 is slowly moved until it reaches the neutral's point 5 position:
    while(analogRead(A1)>X_NOMINAL_2)
    {
//      Serial.print("A3_x<:");      
//      Serial.println(analogRead(A1));
//      
      // Impose a low duty cycle PWM to motor 2:
      motor2.impose_PWM(PWM_RESET_MOTOR_2);
    }
    // Delay to turn off the motor:
    delay(50);
    // Both motors are stopped:
    motor2.turn_off();
  }
}

// Adjacency/cost Matrix representative of the system:
int cost_matrix[9][9]={{5,5,5,1,5,5,5,5,5},
                       {5,5,5,5,1,5,5,5,5},
                       {5,5,5,5,5,1,5,5,5},
                       {1,5,5,5,0,5,1,5,5},
                       {5,1,5,0,5,0,5,1,5},
                       {5,5,1,5,0,5,5,5,1},
                       {5,5,5,1,5,5,5,5,5},
                       {5,5,5,5,1,5,5,5,5},
                       {5,5,5,5,5,1,5,5,5}};

/** n sum_array
 * 
 *  rief Description: This function calculates the sum of all the elements of an array of integers.
 * 
 * \param[in] array The array which elements need to be summed;
 * \param[in] length The size of the array.
 * 
 * \param[out] sum The result of the sum of all the elements in the array.
 * 
 */
int sum_array(int array[],int length)
{
  // Cycle increment and sum result variable:
  int i,sum=0;
  // For cycle to go across the extent of the array:
  for(i=1;i<=length;i++)
  {
    // The sum is constantly accumulated:
     sum=sum+array[i];
   }
   // Returns the value:
   return sum;
}
/** n get_min_index
 * 
 *  rief This function gets the index of the minimum value of a given array.
 * 
 * \param[in] array The array where the minimum value's index will be calculated;
 * \param[in] arraySize The size of the array.
 * 
 * \param[out] index The index of the minimum value in the array.
 * 
 */
int get_min_index(int array[],int arraySize)
{
  // Gets the first value of the array:
  int minim=array[1];
  // And stores that index:
  int index=1;
  // For cycle to go across the extent of the array:
  for(int i=2;i<arraySize;i++)
  {
    // If a new minimum value is found:
    if(minim>array[i])
    {
      // That new minimum value is stored:
      minim=array[i];
      // And its index is also stored:
      index=i;
    }
  }
  // The index is returned:
  return index;
}

/** n reverse_array
 * 
 *  rief This function reverses the elements of a given array.
 * 
 * \param[in] s The array to be inverted. In the end this array (which is consequently also a pointer) will be reversed;
 * \param[in] size_s The size of the array to be inverted.
 * 
 */
void reverse_array(int s[],int size_s)
{
  // Temporary integer value:
  int c;
  // For cycle to go across the extent of the vector, and i is incremented as j is decremented:
  for(int i=0,j=size_s;i<j;i++,j--)
  {
    // The current value of the array is stored:
    c=s[i];
    // That value is stored in vector s:
    s[i]=j;
    // And the last calculated index of s is equalled to c:
    s[j]=c;
  }
}

/** n dijkstra
 * 
 *  rief This function calculates the shortest path between two points on a graph, represented by an adjacency/cost matrix.
 * 
 * \param[in] matrix The adjacency/cost matrix representing the graph;
 * \param[in] sorce The source point;
 * \param[in] destination The destination point;
 * \param[in] size_a Pointer where the size of the array will be written.
 * 
 * \param[out] c The path array, representing the path traveled to go from the source point to the destination point.
 * 
 */
int *dijkstra(int matrix[9][9],int source,int destination,int *size_a)
{
  int u;
  int S[10]={0,0,0,0,0,0,0,0,0,0};
  int dist[10]={5,5,5,5,5,5,5,5,5,5};
  int prev[10]={10,10,10,10,10,10,10,10,10,10};
  dist[source]=0;
  int candidate[10]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
  while(sum_array(S,9)!=9)
  {
    for(int i=1;i<=9;i++)
    {
      if(S[i]==0)
	candidate[i]=dist[i];
      else
	candidate[i]=5;
    }
    u=get_min_index(candidate,10);
    S[u]=1;
    for(int i=1;i<10;i++)
    {
      if(dist[u]+matrix[u-1][i-1]<dist[i])
      {
	dist[i]=dist[u]+matrix[u-1][i-1];
	prev[i]=u;
      }
    }
  }
  // Generation of the final vector:
  Vector<int> sp;
  sp.push_back(destination);
  int counter=0;
  int last=destination;
  while(last!=source)
  {
    if(prev[last]<=9)
      {
        sp.push_back(prev[last]);
        last=prev[last];
      }
  }
  int c[sp.size()];
  // Invertion of the result:
  for(int i=0;i<sp.size();i++)
  {
    c[i]=sp[sp.size()-1-i];
  }
  *size_a=sp.size();
  return c;
}

/** n get_current_point
 * 
 *  rief This function searches the Resistance Table until it finds the point in which the gear lever is currently on. The point returned is normalizes to fit the interval [1,9].
 * 
 * \param[in] res_table The Resistance Table.
 * 
 * \param[out] i The current position [1,9]
 * 
 */
int get_current_point(int res_table[9][6])
{
  // The x potentiometer's value is read:
  int x_res=analogRead(A1);
  // The y potentiometer' value is read:
  int y_res=analogRead(A0);
  // Cycle to search the resistance values' matrix:
  for(int i=0;i<9;i++)
  {
    // If the value is between the defined limits:
    if((x_res>=res_table[i][1])&(x_res<=res_table[i][2])&(y_res>=res_table[i][4])&(y_res<=res_table[i][5]))
    {
      // Return of the normalizes point value:
      return i+1;
    }
    delay(10);
  }
}

/** n point_to_gear
 * 
 *  rief This function converts a point [from 1 to 9] to a gear [from 0 to 6]. If a point has no correspondent gear, the function returns -1;
 * 
 * \param[in] point The point to convert.
 * 
 */
int point_to_gear(int point)
{
  // A Switch-Case is started:
  switch(point)
  {
    // The gear corresponding to point number one:
    case 1:
      // Is the First gear:
      return 1;
    // The gear corresponding to point number two:
    case 2:
      // Is the Third gear:
      return 3;
    // The gear corresponding to point number three:
    case 3:
      // Is the Fifth gear:
      return 5;
    // The gear corresponding to point number four:
    case 4:
      // Is the Neutral:
      return 0;
    // The gear corresponding to point number five:
    case 5:
      // Is the Neutral:
      return 0;
    // The gear corresponding to point number six:
    case 6:
      // Is the Neutral:
      return 0;
     // The gear corresponding to point number seven:
    case 7:
      // Is the Second gear:
      return 2;
    // The gear corresponding to point number eight:
    case 8:
      // Is the Fourth gear:
      return 4;
    // The gear corresponding to point number nine:
    case 9:
      // Is the Sixth gear:
      return 6;
    // And if no correspondence is found:
    default:
      // The function returns -1:
      return -1;
  }
}

/** n gear_to_point
 * 
 *  rief This function converts a gear [from 0 to 6] to a point [from 1 to 9]. If the gear is unknown, the function returns -1.
 * 
 * \param[in] point The gear to convert.
 * 
 */
int gear_to_point(int gear)
{
  // A Switch-Case is started:
  switch(gear)
  {
    // If a Neutral is requested:
    case 0:
      // The corresponding point is point number 5:
      return 5;
    // If a First Gear is requested:
    case 1:
      // The corresponding point is point number 1:
      return 1;
    // If a Second Gear is requested:
    case 2:
      // The corresponding point is point number 7:
      return 7;
    // If a Second Gear is requested:
    case 3:
      // The corresponding point is point number 2:
      return 2;
    // If a Fourth Gear is requested:
    case 4:
      // The corresponding point is point number 8:
      return 8;
    // If a Fifth Gear is requested:
    case 5:
      // The corresponding point is point number 3:
      return 3;
    // If a Sixth Gear is requested:
    case 6:
      // The corresponding point is point number 9:
      return 9;
    // If a point is not contemplated:
    default:
      // The function returns -1:
      return -1;
  }
}

/** n determine_transitions
 * 
 *  rief This function determines the transitions needed to accomplish a determined path, and writes it in the transitions vector. An horizontal path will be represented as a 0, and a vertical path will be represented as a 1. For example, if the path vector is [1,4,5,6,9], the transitions vector would be [1,0,0,1].
 * 
 * \param[in] transitions The vector transition, in which the transitions will be written.
 * \param[in] size_path The size of the path vector.
 * \param[in] path The path vector itself.
 * \param[in] matrix The adjacency/cost matrix.
 * 
 */
void determine_transitions(int *transitions,int size_path,int *path,int matrix[9][9])
{
  // The for will go along the path vector:
  for(int i=0;i<size_path-1;i++)
  {
    // If a Vertical path is found:
    if(matrix[path[i]-1][path[i+1]-1]==0)
    {
      // The corresponding transition is set do a Vertical one:
      transitions[i]=0;
    }
    // And if a Horizontal path is found:
    else if(matrix[path[i]-1][path[i+1]-1]==1)
      // The corresponding transition is set do a Horizontal one:
      transitions[i]=1;
  }
}

/** n multiMap
 * 
 *  rief This function interpolates val linearly, between _in and _out values.
 * 
 * \param[in] val The value to be linearly interpolated, belonging to x;
 * \param[in] _in The x values array;
 * \param[in] _out The y values array;
 * \param[in] size The size of the _in and _out vectors.
 * 
 * \param[out] result The return value of interpolated value, according to the map function.
 * 
 */
int multiMap(int val, int* _in, int* _out, uint8_t size)
{
  // Take care the value is within range:
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];
  // Search right interval:
  uint8_t pos = 1;
  while(val > _in[pos]) pos++;
  // This will handle all exact "points" in the _in array:
  if (val == _in[pos]) return _out[pos];
  // Interpolate in the right segment for the rest:
  return map(val, _in[pos-1], _in[pos], _out[pos-1], _out[pos]);
}

/** n interp_pwm
 * 
 *  rief This function interpolates the PWM to apply to the motor given the current point, the PWM intervals, the ramp width, the starting and ending potentiometer resistance values in the current axis and the PWM limits.
 * 
 * \param[in] current_pot_value The current value read in the potentiometer;
 * \param[in] PWM_min The minimum limit for the PWM;
 * \param[in] PWM_max The maximum limit for the PWM;
 * \param[in] start_pot_value The starting value of the potentiometer;
 * \param[in] start_pot_value The final value of the potentiometer;
 * \param[in] ramp_width The width for theb desired acceleration ramp.
 * 
 * \param[out] result The return value of interpolated value, according to the multiMap function.
 * 
 */
int interp_pwm(int current_pot_value,int PWM_min,int PWM_max,int start_pot_value,int end_pot_value,int ramp_width)
{
  // Generates the 4-point PWM curve:
  int pwm[]={PWM_min,PWM_max,PWM_max,PWM_min};
  // If the starting potentiometer value is bigger than the ending one:
  if(start_pot_value>end_pot_value)
  {
    // Generates the potentiometer's values vector:
    int in[]={end_pot_value,end_pot_value+ramp_width,start_pot_value-ramp_width,start_pot_value};
    // Interpolates and returns the PWM value that must be imposed:
    return multiMap(current_pot_value,in,pwm,4);
  }
  // If the starting potentiometer value is smaller than the ending one:
  else
  {
    // Generates the potentiometer's values vector:
    int in[]={start_pot_value,start_pot_value+ramp_width,end_pot_value-ramp_width,end_pot_value};
    // Interpolates and returns the PWM value that must be imposed:
    return multiMap(current_pot_value,in,pwm,4);
  }
}

/** n isGearValid
 * 
 *  rief This function checks the validity of a gear. Basically it checks if that gear belongs to the interval [0,6].
 * 
 * \param[in] gear The gear that needs to be checked;
 * 
 * \param[out] return 1 if the gear is valid, and 0 otherwise.
 * 
 */
bool isGearValid(int gear)
{
  return gear>=0 && gear<=6;
}

/** n Communication
 * 
 *  rief This function is responsible for the communication with the gear selector mechanism. This function allways generates a response to a question made from the PC via TCP/IP. The protocol followed is divided in two distinct messages, one comming from the PC, and one going to the PC. The message comming from the PC must have the following format: \n <STX> opcode value <ETX> \n If the opcode is "GG" (the function is not case sensitive) the "value" is ignored, and the Arduino only informs about the current gear. The opcode can also assume the string SG (the function is not case sensitive) in which case a gear is set. The gear number is in the "value" part of the message, not ignored in this case. \n The answer has the following format: \n <STX> cg code <ETX> \n The "code" part of the message can assume the following values: \n -> "s" - Informs that the gear lever has started to move; \n -> "c" - Informs that the gear lever has concluded its last movement successfully; \n -> "i" - Indicates that the asked gear is invalid; \n -> "m" - Indicates that the computer tried to set a gear while the system is in manual mode; \n -> "u" - Indicates that a command is unrecognised; \n -> "gear_number" - Indicates the number of the current gear.
 * And the opcode 
 * 
 * \param[in] current_pot_value The current value read in the potentiometer;
 * 
 * \param[out] result The return value of interpolated value, according to the multiMap function.
 * 
 */

void Communication(EthernetClient* client)
{
  // The TCP/IP buffer for the Ethernet connection:
  char buffer[250];
  // The number of bytes received:
  int received_bytes=0;
  // The string which will contain the message from the Arduino to the PC:
  char ar_to_pc_msg[100];
  // If the client is connected:
  if(client->connected())
  {
    // While the client is available and the number of bytes is smaller than the buffer size:
    while(client->available() && received_bytes<sizeof(buffer))
    {
      // The buffer is read:
      buffer[received_bytes++]=client->read();
    }
    Serial.print(buffer);
    
    if(received_bytes==0)//no new message
		return;
		
    
    // Adds the terminator to the message:
    buffer[received_bytes-1]='\0';
    // Variable to store the result of the message scan:
    char opcode[250];
	memset(opcode,0,sizeof(opcode));
    // The value of the new requested gear:
    int value=-1;
    // The buffer is scanned and both opcode and the number of the asked gear:
    sscanf(buffer,"%s %d",opcode,&value);
    // Message interpretation, if the opcode is a "set" code:
    if((opcode[0]=='s' || opcode[0]=='S') && (opcode[1]=='g' || opcode[1]=='G')) 
    {
      // If the mode is set to automatic:
      if(!running_mode)
      {
		  // The validity of the received gear is verified, and if it is valid:
			if(isGearValid(value))
			{
				// The incoming gear global variable is filled with the value in the message:
				incoming_gear=value;
				// A message saying that the change of the gear has started is concatenated:
				sprintf(ar_to_pc_msg,"%ca sg o%c",2,3);
				// And the message is sent:
				SendMessage(client,ar_to_pc_msg);//in auto mode
				// If the current gear is equal to the asked gear:
// 				if(current_gear==incoming_gear)
// 				{
					// A message saying that the change of the gear is done is concatenated:
// 					sprintf(ar_to_pc_msg,"%c cg c %c",2,3);
					// And the message is sent:
// 					SendMessage(client,ar_to_pc_msg);//in auto mode
// 				}
			}
			// If the received gear is not valid:
			else
			{
				// A message saying that the asked gear is not valid is concatenated:
				sprintf(ar_to_pc_msg,"%ca sg i%c",2,3);
				// And the message is sent:
				SendMessage(client,ar_to_pc_msg);//in auto mode
			}
		}
		// If the mode is set to manual, and a gear change is requested:
		else
		{
			// A message saying that the system is not in auto mode is concatenated:
			sprintf(ar_to_pc_msg,"%ca sg m%c",2,3);
			// And the message is sent:
			SendMessage(client,ar_to_pc_msg);//in auto mode
		}
    }
    // Message interpretation, if the opcode is a "get" code:
    else if((opcode[0]=='g' || opcode[0]=='G') && (opcode[1]=='g' || opcode[1]=='G'))
    {
		if(need_change)
		{
			sprintf(ar_to_pc_msg,"%ca gg c %d - %d%c",2,current_gear,received_gear,3);
			SendMessage(client,ar_to_pc_msg);//in auto mode
		}else
		{
			// A message saying the current gear is concatenated:
			sprintf(ar_to_pc_msg,"%ca gg a %d%c",2,current_gear,3);
			// And the message is sent:
			SendMessage(client,ar_to_pc_msg);
		}
	}
	// And finally, if the opcode is not recognized:
    else
    {
		// A message saying that the opcode is not recognized is concatenated:
		sprintf(ar_to_pc_msg,"%ca u%c",2,3);
		// And the message is sent:
		SendMessage(client,ar_to_pc_msg);
    }
  }
}

//Msg must be NULL terminated
void SendMessage(EthernetClient* client,const char*msg)
{
  if(client->connected())
    client->println(msg);
}

void loop(void)
{
//  Serial.print("YY:");
//  Serial.println(analogRead(A0));
//  Serial.print("XX:");
//  Serial.println(analogRead(A1));
//  Serial.print("gear:");
//  Serial.println(current_gear);
//  Serial.print("emergencia:");      
//  Serial.println(digitalRead(A5));   
//  Serial.print("UP:");      
//  Serial.println(digitalRead(A2));
//  Serial.print("MAN/AUTO:");      
//  Serial.println(digitalRead(9));  
//  Serial.print("DOWN:");      
//  Serial.println(digitalRead(8)); 
    
  pinMode(8,INPUT); // Blue (Gear Down) -> Pin 8:
  
  delay(10);
  // Listen for any incoming client:
  EthernetClient client=server.available();
  // While there is no need for a gear change, and the running mode is set to:
  //################################################################################
  //                                                                              //
  //    ####   ##    ## ########  ######  ##    ##   ####   ######## ##  ######   //
  //   ##  ##  ##    ##    ##    ##    ## ###  ###  ##  ##     ##    ## ##    ##  //
  //  ##    ## ##    ##    ##    ##    ## ## ## ## ##    ##    ##    ## ##        //
  //  ######## ##    ##    ##    ##    ## ##    ## ########    ##    ## ##    ##  //
  //  ##    ##  ######     ##     ######  ##    ## ##    ##    ##    ##  ######   //
  //                                                                              //
  //################################################################################
  if((!need_change)&&(!running_mode))
  {
    //Serial.println("Modo Automático");
    // Calculates the current point:
    current_point=get_current_point(resistance_matrix);
    // Calculates the current gear:
    current_gear=point_to_gear(current_point);
//    // The seven segment display holds the current gear value:
//    sevenSegWrite(current_gear,sevenSeg_pins);
    // The mode button is read:
    reading_button_MODE=digitalRead(9);
    // If the MODE button is pressed:
    if(reading_button_MODE==HIGH)
    {
      // The running mode is changes to AUTO:
      running_mode=1;
      delay(1000);
    }
    // If the TCP Client is available, the message is sent to the PC:
    Communication(&client);
    //Serial.println(current_gear);
    if((incoming_gear!=current_gear)&&(incoming_gear>-1)&&(incoming_gear<7))
    {
      received_gear=incoming_gear;
      need_change=1;
    }
    else
      return;
  }
  
  // While there is no need for a gear change, and the running mode is set to:
  //###########################################################
  //                                                         //
  //  ##    ##   ####   ##    ## ##    ##   ####   ##        //
  //  ###  ###  ##  ##  ###   ## ##    ##  ##  ##  ##        //
  //  ## ## ## ##    ## ## #  ## ##    ## ##    ## ##        //
  //  ##    ## ######## ##  # ## ##    ## ######## ##        //
  //  ##    ## ##    ## ##   ###  ######  ##    ## ########  //
  //                                                         //
  //###########################################################
  if((!need_change)&&(running_mode))
  {
    //Serial.println("Modo Manual");
    // The DOWN button is read:
    reading_button_DN=digitalRead(8);
    // The UP button is read:
    reading_button_UP=digitalRead(A2);
    // The mode button is read:
    reading_button_MODE=digitalRead(9);
    // Calculates the current point:
    current_point=get_current_point(resistance_matrix);
    // Calculates the current gear:
    current_gear=point_to_gear(current_point);
//    // The seven segment display holds the current gear value:
//    sevenSegWrite(current_gear,sevenSeg_pins);
    // If the UP button is pressed, and the gears are between admissible values:
    if ((reading_button_UP==HIGH)&(current_gear>-1)&(current_gear<7))
    {
      delay(10);
      // The current gear is incremented:
      received_gear=current_gear+1;
      //Serial.println(received_gear);
      if(received_gear==7)
	received_gear=0;
      if(received_gear==6)
      {
	received_gear=5;
	return;
      }
      // The need for a change is set to 1:
      need_change=1;
      delay(10);
    }
    // If the DOWN button is pressed, and the gears are between admissible values:
    else if ((reading_button_DN==HIGH)&(current_gear>-1)&(current_gear<6))
    {
      delay(10);
      // The current gear is decremented:
      received_gear=current_gear-1;
      if(received_gear==-1)
	received_gear=6;
      // The need for a change is set to 1:
      need_change=1;
      delay(10);
    }
    // If the MODE button is pressed:
    else if(reading_button_MODE==HIGH)
    {
      running_mode=0;
      delay(1000);
    }
    Communication(&client);
    return;
  }
  // Calculates the current point, just in the case the while is inactivated:
  current_point=get_current_point(resistance_matrix);
//  // Writes a seven in the Seven Segment display:
//  sevenSegWrite(7,sevenSeg_pins);
  
  if(need_change)
  {
    // The next point is calculated:
    int next_point=gear_to_point(received_gear);
    // Array to store the path:
    int *path;
    // Variable to store the size of that path array:
    int size_path;
    // Applying the Dijkstra Method to determine the path:
    path=dijkstra(cost_matrix,current_point,next_point,&size_path);
    delay(10);
    // Array to store the transitions:
    int transitions[size_path-1];
    // Determination of the current transitions:
    determine_transitions(transitions,size_path,path,cost_matrix);
    //####################################
    // The purpose of this code section is to correct the path and transition vectors, so that the motors don't stop on a node that is located between two point in the same direction:
    //####################################
    // The new corrected path vector:
    Vector<int> new_path;
    // The new corrected transitions vector:
    Vector<int> new_transitions;
    
    new_transitions.push_back(transitions[0]);
    new_path.push_back(path[0]);
    for(int i=0;i<size_path-2;i++)
    {
      if (transitions[i]!=transitions[i+1])
      {
	new_path.push_back(path[i+1]);
	new_transitions.push_back(transitions[i+1]);
      }
    }
    new_path.push_back(path[size_path-1]);
    //####################################
    
    
    // the transitions vector will be travelled until the last point is reached:
    for (int i=0;i<new_transitions.size();i++)
    {
      // If the path is vertical, ergo the movement must be made in Y by motor 1:
      if(new_transitions[i]==1)
      {
	// The motor 1 is assured to be stopped:
	motor1.turn_off();
	// A zero duty cycle is imposed:
	motor1.impose_PWM(0);
	// If the next point in the path is greater than the previous one, the motor 1 must move in the positive Y direction:
	if(new_path[i]<new_path[i+1])
	{
	  // The analogue port is read:
	  int a=analogRead(A0);
	  // The starting value is saved:
	  int start_pot_value=a;
	  // The ending analogue value is read from the resistance table:
	  int end_pot_value=resistance_matrix[new_path[i+1]-1][3];
	  // The motor 1 is turned on clockwise:
	  motor1.turn_on_clockwise();
	  // While the resistance value is not the one defined in the table:
          unsigned int cont=0;
//          while(a<end_pot_value)
	  while(a>end_pot_value)
	  {
	    // If the TCP Client is available, the message is sent to the PC:
	    Communication(&client);
// 	    char tmp[100];
// 	    sprintf(tmp,"%c cg %d - %d %c",2,current_gear,received_gear,3);
// 	    SendMessage(&client,tmp);
	    // The analogue port is read:
	    a=analogRead(A0);
	    // An interpolated PWM value is imposed:
	    motor1.impose_PWM(interp_pwm(a,PWM_MIN_MOTOR_1,PWM_MAX_MOTOR_1,start_pot_value,end_pot_value,20));
	  }
	  // Delay to turn off the motor:
	  //delay(50);
	  // Both motors are stopped:
	  motor1.turn_off();
	}
	// If the next point in the path is smaller than the previous one, the motor 1 must move in the negative Y direction:
	else if(new_path[i]>new_path[i+1])
	{
	  // The analogue port is read:
	  int a=analogRead(A0);
	  // The starting value is saved:
	  int start_pot_value=a;
	  // The ending analogue value is read from the resistance table:
	  int end_pot_value=resistance_matrix[new_path[i+1]-1][3];
	  // The motor 1 is turned on counter clockwise:
	  motor1.turn_on_counterclockwise();
	  // While the resistance value is not the one defined in the table:
//	  while(a>end_pot_value)
          while(a<end_pot_value)
	  {
	    // If the TCP Client is available, the message is sent to the PC:
	    Communication(&client);
// 	    char tmp[100];
// 	    sprintf(tmp,"%c cg %d - %d %c",2,current_gear,received_gear,3);
// 	    SendMessage(&client,tmp);
	    // The analogue port is read:
	    a=analogRead(A0);
	    // An interpolated PWM value is imposed:
	    motor1.impose_PWM(interp_pwm(a,PWM_MIN_MOTOR_1,PWM_MAX_MOTOR_1,start_pot_value,end_pot_value,20));
	  }
	  // Delay to turn off the motor:
	  delay(50);
	  // Both motors are stopped:
	  motor1.turn_off();
	}
      }
      
      // If the path is horizontal, ergo the movement must be made in X by motor 2:
      else if(new_transitions[i]==0)
      {
	// The motor 2 is assured to be stopped:
	motor2.turn_off();
	// A zero duty cycle is imposed:
	motor2.impose_PWM(0);
	// If the next point in the path is greater than the previous one, motor 2 must move in the positive X direction:
	if(new_path[i]<new_path[i+1])
	{
	  // The analogue port is read:
	  int a=analogRead(A1);
	  // The starting value is saved:
	  int start_pot_value=a;
	  // The ending analogue value is read from the resistance table:
	  int end_pot_value=resistance_matrix[new_path[i+1]-1][0];
	  // The motor 2 is turned on counter clockwise:
	  motor2.turn_on_counterclockwise();
	  // While the resistance value is not the one defined in the table:
//          while(a<end_pot_value)
	  while(a>end_pot_value)
	  {
	    // If the TCP Client is available, the message is sent to the PC:
	    Communication(&client);
// 	    char tmp[100];
// 	    sprintf(tmp,"%c cg %d - %d %c",2,current_gear,received_gear,3);
// 	    SendMessage(&client,tmp);
	    // The analogue port is read:
	    a=analogRead(A1);
	    // An interpolated PWM value is imposed:
	    motor2.impose_PWM(interp_pwm(a,PWM_MIN_MOTOR_2,PWM_MAX_MOTOR_2,start_pot_value,end_pot_value,15));
	  }
	  
	  // Delay to turn off the motor:
	  delay(50);
	  // Both motors are stopped:
	  motor2.turn_off();
	}
	// If the next point in the path is smaller than the previous one, motor 2 must move in the negative X direction:
	else if(new_path[i]>new_path[i+1])
	{
	  // The analogue port is read:
	  int a=analogRead(A1);
	  // The starting value is saved:
	  int start_pot_value=a;
	  // The ending analogue value is read from the resistance table:
	  int end_pot_value=resistance_matrix[new_path[i+1]-1][0];
	  // The motor 1 is turned on counter clockwise:
	  motor2.turn_on_clockwise();
	  // While the resistance value is not the one defined in the table:
//          while(a>end_pot_value)
	  while(a<end_pot_value)
	  {
	    // If the TCP Client is available, the message is sent to the PC:
	    Communication(&client);
// 	    char tmp[100];
// 	    sprintf(tmp,"%c cg %d - %d %c",2,current_gear,received_gear,3);
// 	    SendMessage(&client,tmp);
	    // The analogue port is read:
	    a=analogRead(A1);
	    // An interpolated PWM value is imposed:
	    motor2.impose_PWM(interp_pwm(a,PWM_MIN_MOTOR_2,PWM_MAX_MOTOR_2,start_pot_value,end_pot_value,15));
	  }
	  // Delay to turn off the motor:
	  delay(50);
	  // Both motors are stopped:
	  motor2.turn_off();
	  // Delay to turn off the motor:
	  delay(50);
	}
      }
    }
    
//     char tmp[100];
//     sprintf(tmp,"%c cg done %c",2,3);
//     SendMessage(&client,tmp);
    
    need_change=0;
  }
}	
