#include<SoftwareSerial.h>

SoftwareSerial rpi(2, 3); // SoftwareSerial(RX, TX): UNO pin 2 is RX (connected to TX of RPi), 3 is TX (to RPi RX)

int getLatestCmd();
int instantCmd();
boolean timer();
void executeCmd(int);
void forward();
void backward();
void left();
void right();
void stopMotion();
void defaultState();
void continuePrevious();
void dance();
void namaste();
void unknownCmd(int);



int prev_cmd_id;
int current_state_id;

void setup(){
	// Motor pins
	
	pinMode(4, OUTPUT); // motor1_1
	pinMode(5, OUTPUT); // motor1_2
	pinMode(6, OUTPUT); // motor2_1
	pinMode(7, OUTPUT); // motor2_2

	pinMode(8, OUTPUT); // Buzzer

	pinMode(9, OUTPUT); // Ultra sonic 1 hc8 sr04 for front trig pin
	pinMode(11, OUTPUT); // Ultra sonic 2 hc sr04 for ground trig pin

	pinMode(10, INPUT); // Ultra sonic 1 hc sr04 for front echo pin
	pinMode(12, INPUT); // Ultra sonic 2 hc sr04 for ground echo pin

	Serial.begin(9600);
	rpi.begin(9600);

	prev_cmd_id = 1;
	current_state_id = 1;
}

void loop(){
	int cmd_id = 2; // 2 id means continue previous task
	if(timer())
	{
		cmd_id = getLatestCmd();
	}else{
		cmd_id = instantCmd();
	}
	if(cmd_id!=2 && cmd_id!=-1){
		prev_cmd_id=cmd_id;
	}

	// checking obstacles
	// front obstacle check
	boolean obs_in_front = obstacleInFront();
	
	// ground check
	boolean pit_in_gnd = pitInGround();

	// If obstacle in front or pit in gnd then stop
	if(obs_in_front || pit_in_gnd){
		cmd_id = 0; // stop moving
    digitalWrite (8, HIGH);
	}	else{
     digitalWrite (8, LOW);}
	
	// execution of cmd
	executeCmd(cmd_id);
}

boolean obstacleInFront(int min_dis_in_cm=25){
int average = 0;
for(int i=0; i<5;i++){

	// checking obstacles in front
	digitalWrite(9, LOW);
	delayMicroseconds(5);
	digitalWrite(9, HIGH);
	delayMicroseconds(10);
	digitalWrite(9, LOW);

	int dis_in_cm = (pulseIn(10, HIGH, 30000)/2)*0.0343;
average+=dis_in_cm;
}
average= average/5;
	if(average< min_dis_in_cm){
		return true;
	}
	return false;
}

boolean pitInGround(int max_dis_in_cm = 3){
int average=0;
for(int i=0; i<5;i++){
	// checking pit in ground
	digitalWrite(11, LOW);
	delayMicroseconds(5);
	digitalWrite(11, HIGH);
	delayMicroseconds(10);
	digitalWrite(11, LOW);
	
	int dis_in_cm = (pulseIn(12, HIGH, 30000)/2)*0.0343;
average+= dis_in_cm;
}
average/=5;
	if(average > max_dis_in_cm)
	{
		return true;
	}
	return false;
}

boolean timer(){
	// Returns true for every 5 seconds passed
  static unsigned long lastCheck = 0;
	if(millis()-lastCheck >= 5000){
    lastCheck = millis();
		return true;
	}
	return false;
}

// Gives current state of this node and asks if there is new cmd to switch on
int getLatestCmd(){
	int cmd_id = -1; // -1 id means no response from rpi
	rpi.println("pc="+String(prev_cmd_id)+"|cs="+String(current_state_id)+"|nc=?"); // pc is previous cmd, cs is current stare, nc is next cmd

	unsigned long timeout = millis()+2000; // Maximum wait this node waits for the response from rpi

	String response = "";
	
	while(millis() < timeout){
		if(rpi.available()){
			char c = rpi.read();
			response += c;
			if(c=='\n') break;
		}
	}

	response.trim(); // remove trailing spaces or newline
	if(response.length()>0){
		cmd_id = response.toInt();
	}

	// response is in the form of integer only which is nothing but should be cmd id
	return cmd_id;
}

// This node also checks if there is unscheduled cmd given to it by rpi
int instantCmd(){
	int cmd_id = 2;

	unsigned long timeout = millis()+200; // timeout for this function is less than getLatestCmd

	String response = "";
	
	while(millis() < timeout){
		if(rpi.available()){
			char c = rpi.read();
			response += c;
			if(c=='\n') break;
		}
	}

	response.trim(); // remove trailing spaces or newline
	if(response.length()>0){
		cmd_id = response.toInt();
	}

	return cmd_id;
}

// This function executes corresponding function according to cmd-id
void executeCmd(int cmd_id){
	switch(cmd_id){
		case 0:
		stopMotion(); // stop both motors
		break;
		
		case 1:
		defaultState(); // go in default state
		break;

		case 2:
		continuePrevious(); // continue doind previous cmd
		break;

		case 3:
		forward(); // forward motion
		break;

		case 4:
		backward(); // backward motion
		break;

		case 5:
		left(); // left motion
		break;

		case 6:
		right(); // right motion
		break;

		case 7:
		dance(); // performs dance steps for legs, hands are handled by nano
		break;

		case 8:
		namaste(); // do namaste posture  for legs, handa are handeled by nano
		break;

		default:
		unknownCmd(cmd_id); // handles unkown cmd id
	}
}

// dance posture of legs
void dance(){
	stopMotion();
	current_state_id = 7;
	delay(100);
	left();
	delay(600);
	right();
	delay(1200);
	left();
	delay(600);
	stopMotion();
}

// namaste posture of legs
void namaste(){
	backward();
	current_state_id = 8;
	delay(300);
	forward();
	delay(300);
	stopMotion();
}

void unknownCmd(int cmd_id){
	Serial.println("Unknown command id : "+String(cmd_id));
	defaultState();
}

void continuePrevious(){
	executeCmd(prev_cmd_id);
}

void defaultState(){
	// Stop doing motion, turn off both motors
	stopMotion();

	// Stop Buzzer, turn off buzzer
	digitalWrite(8, LOW);

	// set current_state_id
	current_state_id = 1;
}

void stopMotion(){
	// Stop doing motion , turn off both motors
	digitalWrite(4, LOW);
	digitalWrite(5, LOW);
	digitalWrite(6, LOW);
	digitalWrite(7, LOW);

	// set current_state_id
	current_state_id = 0;
}

void forward(){
	// Move Forward

	stopMotion();
	delay(100);
	
	digitalWrite(4, HIGH);
	digitalWrite(5, LOW);
	digitalWrite(6, HIGH);
	digitalWrite(7, LOW);

	// set current_state_id
	current_state_id = 3;
}

void backward(){
	// Move backward

	stopMotion();
	delay(100);
	
	digitalWrite(4, LOW);
	digitalWrite(5, HIGH);
	digitalWrite(6, LOW);
	digitalWrite(7, HIGH);

	// set current_state_id
	current_state_id = 4;
}

void left(){
	// Move left

	stopMotion();
	delay(100);
	
	digitalWrite(4, LOW);
	digitalWrite(5, HIGH);
	digitalWrite(6, HIGH);
	digitalWrite(7, LOW);

	// set current_state_id
	current_state_id = 5;
}

void right(){
	// Move right

	stopMotion();
	delay(100);
	
	digitalWrite(4, HIGH);
	digitalWrite(5, LOW);
	digitalWrite(6, LOW);
	digitalWrite(7, HIGH);

	// set current_state_id
	current_state_id = 6;
}
