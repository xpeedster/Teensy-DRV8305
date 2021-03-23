/**
 * Comprehensive BLDC motor control example using encoder and the DRV8302 board
 * 
 * Using serial terminal user can send motor commands and configure the motor and FOC in real-time:
 * - configure PID controller constants
 * - change motion control loops
 * - monitor motor variabels
 * - set target values
 * - check all the configuration values 
 * 
 * check the https://docs.simplefoc.com for full list of motor commands
 * 
 */
#include <SimpleFOC.h>
#include <SPI.h>

// DRV8302 pins connections
// don't forget to connect the common ground pin
#define INH_A 22
#define INH_B 9
#define INH_C 6

#define cs 24
#define nFAULT 25
#define EN_GATE 26
#define M_PWM A1 
#define M_OC A2
#define OC_ADJ A3

bool faultTrig = false;

//#####_TIME MANAGEMENT_#####
float runTime, prevT = 0, timeDif, stateT;
int timeInterval = 1000, totalTempTime;

// Motor instance
BLDCMotor motor = BLDCMotor(4);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C);

// encoder instance
Encoder encoder = Encoder(29, 30, 4000);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  pinMode(nFAULT, INPUT);
  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, LOW);
   
  //SPI start up
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);

  //Motor driver initialization
  delay(250);
  Serial.println("DRV8305 INIT");
  drv_init();
  delay(500);

  Serial.println("enGate Enabled");
  digitalWrite(EN_GATE, HIGH);
  delay(500);

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB); 
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // DRV8302 specific code
  // M_OC  - enable overcurrent protection
  //pinMode(M_OC,OUTPUT);
  //digitalWrite(M_OC,LOW);
  // M_PWM  - enable 3pwm mode
  //pinMode(M_PWM,OUTPUT);
  //digitalWrite(M_PWM,HIGH);
  // OD_ADJ - set the maximum overcurrent limit possible
  // Better option would be to use voltage divisor to set exact value
  //pinMode(OC_ADJ,OUTPUT);
  //digitalWrite(OC_ADJ,HIGH);


  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);


  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set control loop type to be used
  motor.controller = MotionControlType::torque;

  // contoller configuration based on the controll type 
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // default voltage_power_supply
  motor.voltage_limit = 12;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // angle loop controller
  motor.P_angle.P = 20;
  // angle loop velocity limit
  motor.velocity_limit = 50;

  // use monitoring with serial for motor init
  // monitoring port
  //Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 2;

  // define the motor id
  command.add('A', onMotor, "motor");

  Serial.println(F("Full control example: "));
  Serial.println(F("Run user commands to configure and the motor (find the full command list in docs.simplefoc.com) \n "));
  Serial.println(F("Initial motion control loop is voltage loop."));
  Serial.println(F("Initial target voltage 2V."));
  
  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if tatget not set in parameter uses motor.target variable
  motor.move();

  // user communication
  command.run();

  //float e = encoder.getAngle();
  //Serial.println(e,4);

  //Time managment
  runTime = micros();
  timeDif = runTime - prevT;
  prevT = runTime;
  stateT += timeDif;

  //Functions inside this "if" will execute at a 10hz rate
  if(stateT >= 200000){
    stateT = 0;
    //tempStatus();
    faultStatus();
    //Serial.print(temp);
    //Serial.print(", ");
    //Serial.print(a1);
    //Serial.print(", ");
    //Serial.print(a2);
    //Serial.print(", ");
    //Serial.println(a3);
  }
}

//Fault status and manager for the DRV8305
//Datahseet pages 37 and 38
void faultStatus(){
  //Read nFault pin from DRV8305 - LOW == error / HIGH == normal operation
  int fault = digitalRead(nFAULT);
  
  if(fault == LOW && faultTrig == false){
    Serial.println("Fault detected");
    faultTrig = true;
    //Check warning and watchdog reset (Address = 0x1)
    digitalWrite(cs, LOW);
    byte ft1 = SPI.transfer(B10001000);
    byte ft2 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x1");
    Serial.println(ft1,BIN);
    Serial.println(ft2,BIN);

    //Check OV/VDS Faults (Address = 0x2)
    digitalWrite(cs, LOW);
    byte ft3 = SPI.transfer(B10010000);
    byte ft4 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x2");
    Serial.println(ft3,BIN);
    Serial.println(ft4,BIN);

    //Check IC Faults (Address = 0x3)
    digitalWrite(cs, LOW);
    byte ft5 = SPI.transfer(B10011000);
    byte ft6 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x3");
    Serial.println(ft5,BIN);
    Serial.println(ft6,BIN);

    //Check VGS Faults (Address = 0x4)
    digitalWrite(cs, LOW);
    byte ft7 = SPI.transfer(B10100000);
    byte ft8 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x4");
    Serial.println(ft7,BIN);
    Serial.println(ft8,BIN);
  }
}

//Configure DRV8305 to desired operation mode
void drv_init(){

  //Set to three PWM inputs mode
  digitalWrite(cs, LOW);
  byte resp1 = SPI.transfer(B00111010);
  byte resp2 = SPI.transfer(B10000110);
  digitalWrite(cs, HIGH);

  Serial.println(resp1, BIN);
  Serial.println(resp2, BIN);
  
}

//Configure DRV8305 to desired operation mode
void drv_read(){

  //Set to three PWM inputs mode
  digitalWrite(cs, LOW);
  byte resp1 = SPI.transfer(B10111010);
  byte resp2 = SPI.transfer(B10000110);
  digitalWrite(cs, HIGH);

  Serial.println(resp1, BIN);
  Serial.println(resp2, BIN);
  
}
