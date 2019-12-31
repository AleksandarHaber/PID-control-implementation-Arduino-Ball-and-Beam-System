//sensor parameters

int distanceSensorPin = A0;   // distance sensor pin
float Vr=5.0;                 // reference voltage for A/D conversion
float sensorValue = 0;        // raw sensor reading
float sensorVoltage = 0;      // sensor value converted to volts 
float k1=16.7647563;          // sensor parameter fitted using the least-squares method
float k2=-0.85803107;         // sensor parameter fitted using the least-squares method
float distance=0;             // distance in cm       
int noMeasurements=200;        // number of measurements for averaging the distance sensor measurements 
float sumSensor;              // sum for computing the average raw sensor value

// motor parameters
#include <Servo.h>
Servo servo_motor; 
int servoMotorPin = 9;        // the servo motor is attached to the 9th Pulse Width Modulation (PWM)Arduino output


// control parameters
float desiredPosition=35;     // desired position of the ball
float errorK;                 // position error at the time instant k
float errorKm1=0;             // position error at the time instant k-1
float errorKm2=0;             // position error at the time instant k-2
float controlK=0;             // control signal at the time instant k
float controlKm1=0;           // control signal at the time instant k-1
int delayValue=0;             // additional delay in [ms]

float Kp=0.2;                        // proportional control 
float Ki=10;                         // integral control
float Kd=0.4;                        // derivative control
float h=(delayValue+32)*0.001;       // discretization constant, that is equal to the total control loop delay, the number 32 is obtained by measuring the time it takes to execute a single loop iteration                         

float keK=Kp*(1+h/Ki+Kd/h);               // parameter that multiplies the error at the time instant k
float keKm1=-Kp*(1+2*Kd/h);               // parameter that multiplies the error at the time instant k-1                    
float keKm2=Kp*Kd/h;                      // parameter that multiplies the error at the time instant k-2

void setup() 
{
  Serial.begin(9600);
  servo_motor.attach(servoMotorPin);
}

void loop() 
{
  unsigned long startTime = micros(); // this is used to measure the time it takes for the code to execute
  // obtain the sensor measurements
  sumSensor=0;

  // this loop is used to average the measurement noise
  for (int i=0; i<noMeasurements; i++)
  {
    sumSensor=sumSensor+float(analogRead(distanceSensorPin));  
  }
  sensorValue=sumSensor/noMeasurements;
  sensorVoltage=sensorValue*Vr/1024;
  distance = pow(sensorVoltage*(1/k1), 1/k2); // final value of the distance measurement

  
  errorK=desiredPosition-distance; // error at the time instant k;

  // compute the control signal
  controlK=controlKm1+keK*errorK+keKm1*errorKm1+keKm2*errorKm2;

  // update the values for the next iteration
  controlKm1=controlK;
  errorKm2=errorKm1;
  errorKm1=errorK;

  servo_motor.write(94+controlK); // the number 94 is the control action necessary to keep the ball in the horizontal position
 // Serial.println((String)"Control:"+controlK+(String)"---Error:"+errorK);

 // these three lines are used to plot the data using the Arduino serial plotter 
  Serial.print(errorK);
  Serial.print(" ");
  Serial.println(controlK);
  unsigned long endTime = micros();
  unsigned long deltaTime=endTime-startTime;
 // Serial.println(deltaTime);
  
 // delay(delayValue); // uncomment this to introduce an additional delay
 
}
