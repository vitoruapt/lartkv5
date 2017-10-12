/*

	DIYServo 1.0

	Controller Motor Speed and Movement with Absolute Positioning

	A sketch for turning a standard DC gear motor into a servo
	using a potentiometer.  Can also be used to add finer-grained
	control over existing servos.

	Control over motor :
		Speed (PWM)
		Direction
		Number of degrees to move (up to 1024) per move
		How to long to wait between moves

	To get finer control (1024 degrees, instead of 360) of an existing
	servo, remove the controller and any stop-pins, disconnect the
	potentiometer wires.  Connect power lines from servo motor to a
	DC motor controller, potentiometer wiper to an input pin on the arduino,
	and the ouside potentiometer pins to 5v/GND.  

	Serial monitor:

		use 'l' to tell the servo to move left
		use 'r' to tell the servo to move right
		use 's' to stop the servo
		use 'g' to go (run the servo)

	(c) 2008 C. A. Church - www.dronecolony.com

	7/24/08 - initial version

*/

	// USING_CONTROLLER says whether we have to bring an
	// enable pin high (such as for the Compact L298 controller)
	// before sending PWM down the LT/RT pins
	// set this to 0 if you don't need an enable pin

#define USING_CONTROLLER 1

	// enable pin
#define MOTOR_EN_PIN     7
	// right direction pin
#define MOTOR_RT_PIN     6
	// left direction pin
#define MOTOR_LT_PIN     5

	// READ_AVG is how many readings to average
	// set it to one less than the actual #
	// e.g.: 10 readings = set to 9
	//
	// the more you average, the more accurate your reading is likely to
	// be -- too many though, and you'll start missing changes if the motor
	// is moving quickly

#define READ_AVG       9

	// motor speed is from 0-255, test with low values
	// as not all will move consistently for you

int motor_speed    = 75;

	// how many ms to pause between allowed movements

int motor_pause_tm = 1000;

	// how many 'degrees' (absolute differences between
	// potentiometer readings) to move before pausing

int motor_move_deg = 5;

	// a counter for how many degrees we have moved

int move_deg_cnt   = 0;

	// setting to a default value

int  motor_cur_pin = MOTOR_RT_PIN;

	// control indicators
bool motor_started = false;
bool motor_paused  = false;
bool first_run     = true;
bool motor_run     = false;

long paused_tm;

	// our current and previous potentiometer readings

int cur_reading = 0;
int pre_reading = 0;

int steps   = 0;

	// our current readings array, and our previous average readings array

int vals[READ_AVG + 1];
int prev_posts[2]  = { 0, 0 };

void setup() {

  Serial.begin(9600);
  Serial.println("Ready");

  memset( (char *)vals, 0, sizeof(int) * (READ_AVG + 1) );

  	// set motor control pins

  if( USING_CONTROLLER )
      digitalWrite(MOTOR_EN_PIN, HIGH);

  digitalWrite(MOTOR_LT_PIN, LOW);
  digitalWrite(MOTOR_RT_PIN, LOW);

}

void loop() {

	// see if any input has come in the serial port

   check_input();

   	// figure out how many degrees we've moved (if at all)

   /*move_deg_cnt = move_deg_cnt + read_pot();

   if( motor_run == true ) {
        // if the motor is supposed to be running

	// the following check is to prevent attempting to rotate all the
	// way around on a potentiometer that has a stop.  If yours doesn't
	// have a stop in it, ou can remove this check

     if( (motor_cur_pin == MOTOR_RT_PIN && prev_posts[1] >= 1020 ) ||
         ( motor_cur_pin == MOTOR_LT_PIN && prev_posts[1] <= 3   ) ) {

		 // we've reached our maximum point (don't want to harm our
		 // potentiometer

       motor_run     = false;
       motor_started = false;
       motor_paused  = false;

       		// bring pin low

       digitalWrite(motor_cur_pin, LOW);

       		// print status

       Serial.println("Have Reached Edge of Movement");
     }
     else if( motor_started == false ) {
	     // the motor is supposed to be running, but we haven't started
	     // it yet

	     // PWM output

       analogWrite(motor_cur_pin, motor_speed);

       	     // set status values

       motor_started = true;
       motor_paused  = false;
       first_run     = true;
     }
     else if( move_deg_cnt >= motor_move_deg && motor_paused == false && first_run == false) {

	     // we've gone our specific # of degrees, pause by stopping the
	     // motor

       Serial.println("Pausing");

       digitalWrite(motor_cur_pin, LOW);
       motor_paused = true;

       		// record when we started our pause (so we know when to stop)

       paused_tm     = millis();
     }
     else if( motor_paused == true && (millis() - paused_tm) > motor_pause_tm ) {

	     	// if enough time has passed to stop pausing

	     Serial.println("Unpausing");
	     motor_paused = false;
	     paused_tm     = millis();

	     	// set move_deg_cnt to zero when re-starting to avoid any
		// jitter while paused

	     move_deg_cnt = 0;

	     	// generate PWM
	     analogWrite(motor_cur_pin, motor_speed);

     }
   }*/

}

int read_pot() {

//read the voltage on the potentiometer:
  cur_reading = analogRead(0);
  int diff = 0;

  	// we're going to average the last READ_AVG reads
	// put in a value for our current step

  vals[steps] = cur_reading;

  	// if we've saved enough values to go ahead and perform an average...

 if( steps == READ_AVG ) {

	 // reset our read counter

   steps = -1;

   	// determine the average value read
	// -- this is mostly to deal with big jitter

   int tot = 0;
   int avg = 0;

   	// sum up totals

   for (int i = 0; i <= READ_AVG; i++)
     tot += vals[i];

   avg = tot / READ_AVG + 1;

   	// ignore current reading if it was either of our last two readings
	// avoid bouncing back and forth between two readings (slight voltage
	// variation in the same range)

   if( avg == prev_posts[0] || avg == prev_posts[1] ) {
	  return(0);
   }

   	// determine the absolute difference between the current average
	// and the previous average

   diff = avg > prev_posts[1] ? avg - prev_posts[1] : prev_posts[1] - avg;

   	// if there's a difference between the averages

   if( diff > 0 ) {

	// print our new reading

    Serial.println(avg, DEC);

  	// move our last reading back, and put our current reading in
	// our array to track the last two positions

    prev_posts[0] = prev_posts[1];
    prev_posts[1] = avg;

	// update this so the pause check knows that we have changed a position
	// (otherwise, starting in a position oher than 0 will mess up our
	// pause check)

    first_run = false;
   }

 }

 	// increment our saved value # for the next loop

 steps++;

	// return the difference recorded
 return(diff);

}

void check_input() {
//  Serial.println("checking input");
  if ( Serial.available()) {
    char ch = Serial.read();

    switch(ch) {
      case 'g':
        Serial.println("Go - Running Motor");
        motor_run = true;
        digitalWrite(7, HIGH);
        break;
      case 's':
        Serial.println("Stopping Motor");
        motor_run = false;
        motor_started = false;
        analogWrite(motor_cur_pin, 0);
        digitalWrite(7, LOW);
        break;
      case 'l':
        motor_cur_pin = MOTOR_LT_PIN;
        Serial.println("Direction = LEFT");
        break;
      case 'r':
        motor_cur_pin = MOTOR_RT_PIN;
        Serial.println("Direction = RIGHT");
        break;
    }
  }
}

