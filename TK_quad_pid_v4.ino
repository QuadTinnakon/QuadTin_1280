//Write by tinnakon ,  TKquadrotor email tinnakon_za@hotmail.com
//23/12/55  write pid  Flight Modes  1.Stabilize  2.Aero and filter romote
//25/12/55  write pid and Flight Modes 3. flip 3d by exponential romote Rate feedback
//30/12/55  write integrated 0.5(w1-w2)dt + w2dt

/*
Quad-X
       
      FRONTL  M1CW        M2CCW  FRONTR
              \         / 
                \ --- /
                 |   |
                / --- \
              /         \ 
    REARL  M4 CCW      M3 CW  REARR
*/

#include <Wire.h>
#include <Servo.h> 
   

//QuadRoror PID GAINS 
#define KP_RATE_ROLL 0.35 //0.285 - 0.855
#define KD_RATE_ROLL 0.031 //0.035
#define KI_RATE_ROLL 2.15 //2.37 2.15

#define KP_RATE_PITCH 0.35 //0.285 - 0.855
#define KD_RATE_PITCH 0.031 //0.035
#define KI_RATE_PITCH 2.25 //2.37 2.15

#define KP_RATE_YAW 2.95 //2.56    2.951
#define KD_RATE_YAW 0.002// 0.015  0.821
#define KI_RATE_YAW 4.62 //1.215   1.821  3.62

#define tar 0.0812 //diff and filter (tar = time constant)//0.75  0.081 0.051  0.0212 0.0612 0.0812
//remote
//#define kprate 0.352  //z 0.352
#define xp1 0.0151 //remote
#define yp1 0.0151
#define xd1 0.312 //remote
#define yd1 0.312
#define zd1 0.451

#define RAD_TO_DEG 57.295779513082320876798154814105
#define MINCOMMAND 1000

Servo motor1;  // create motor
Servo motor2;  // create motor
Servo motor3;  // create motor
Servo motor4;  // create motor

// reading a PWM signal using interrupts
//numbers 0 (on digital pin 2) and 1 (on digital pin 3)
//numbers 2 (pin 21), 3 (pin 20), 4 (pin 19), and 5 (pin 18).
int pin1 = 18;
int pin2 = 19;
int pin3 = 2;
int pin4 = 3; 
int pin5 = 15;
int intrnum1 = 5;
int intrnum2 = 4;
int intrnum3 = 0;
int intrnum4 = 1;
volatile int width1;  // width of most recent signal
volatile unsigned long start1;  // start time of rising signal
volatile int width2;
volatile unsigned long start2;
volatile int width3;
volatile unsigned long start3;
volatile int width4;
volatile unsigned long start4;
volatile int width5;
volatile unsigned long start5;

float roll=1500.0;
float pitch=1500.0;
float yaw=1500.0;
float thro=1060.0;
float aux1=1500.0;

float gyroxtrim=0.0;
float gyroytrim=0.0;
float gyroztrim=0.0;
float xtrim=0.0;
float ytrim=0.0;
float ztrim=0.0;

float xAngle=0.0;
float yAngle=0.0;
float zAngle=0.0;

float roll_I2=0.0;
float roll_D2=0.0;
float err_roll_old2=0.0;

float pitch_I2=0.0;
float pitch_D2=0.0;
float err_pitch_old2=0.0;  //z

float yaw_I=0.0;
float yaw_D=0.0;
float err_yaw_old=0.0;

//Used for timing
unsigned long timer=0;
unsigned long dtime=0;
float dt=0.01;
int i=0;
int k=0;
int armed=0;
// **************************
// I2C Sensor 
// **************************

#define GYR_ADDRESS (0xD2 >> 1) //L3G4200D
#define MAG_ADDRESS (0x3C >> 1)
#define acc_address (0x30 >> 1)
//#define ACC_ADDRESS_SA0_A_HIGH (0x32 >> 1)

float GyroX,GyroY,GyroZ,AccX,AccY,AccZ;
float MagX,MagY,MagZ; //triple axis data

void GyroInit()
{
  //0x8F = 400Hz cut off 20   0xCF = 800Hz cut off 30
	writeReg(0x20, 0x5F);//L3G4200D_CTRL_REG1   0x4F = 200Hz 12.5 0x5F = 200Hz 25 0x0F = 100Hz 12.5  0x1F = 100Hz 25 
        writeReg(0x21, 0x00);//L3G4200D_CTRL_REG2
        writeReg(0x23, 0x00);//L3G4200D_CTRL_REG4 500 0x10 2000 0x20
}

// Writes a gyro register
void writeReg(byte reg, byte value)
{
	Wire.beginTransmission(GYR_ADDRESS);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

// Reads the 3 gyro channels and stores them in vector g
void Gyroread()
{
	Wire.beginTransmission(GYR_ADDRESS);
	// assert the MSB of the address to get the gyro 
	// to do slave-transmit subaddress updating.
	Wire.write(0x28 | (1 << 7)); //L3G4200D_OUT_X_L       0x28
	Wire.endTransmission();
	Wire.requestFrom(GYR_ADDRESS, 6);

	while (Wire.available() < 6);
	
	uint8_t xla = Wire.read();
	uint8_t xha = Wire.read();
	uint8_t yla = Wire.read();
	uint8_t yha = Wire.read();
	uint8_t zla = Wire.read();
	uint8_t zha = Wire.read();

	GyroX = (xha << 8 | xla)*0.00875;
	GyroY = (yha << 8 | yla)*-0.00875;
	GyroZ = (zha << 8 | zla)*0.00875;
}
void AccelerometerInit()
{
    // Enable Accelerometer
  // 0x27 = data 50 Hz // 0x2F data 100 Hz
  // Normal power mode, all axes enabled
  writeAccReg(0x20, 0x27);//LSM303_CTRL_REG1_A
  
  // Enable Magnetometer
  // 0x00 = 0b00000000
  // Continuous conversion mode
  writeMagReg(0x02, 0x00);//LSM303_MR_REG_M
  writeMagReg(0x00, 0x1C);//LSM303_CRA_REG_M    0x1C data 220 hz
  writeMagReg(0x01, 0xE0);//LSM303_CRB_REG_M    0xE0 data +- 8.1 Gauss
}
// Writes an accelerometer register
void writeAccReg(byte reg, byte value)
{
  Wire.beginTransmission(acc_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
// Writes a magnetometer register
void writeMagReg(byte reg, byte value)
{
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
// Reads the 3 accelerometer channels and stores them in vector a
void readAcc(void)
{
  Wire.beginTransmission(acc_address);
  // assert the MSB of the address to get the accelerometer 
  // to do slave-transmit subaddress updating.
  Wire.write(0x28 | (1 << 7)); //LSM303_OUT_X_L_A
  Wire.endTransmission();
  Wire.requestFrom(acc_address, 6);
  while (Wire.available() < 6);
  
  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  AccY = ((xha << 8 | xla) >> 4)*0.01;
  AccX = ((yha << 8 | yla) >> 4)*-0.01;
  AccZ = ((zha << 8 | zla) >> 4)*0.01;
}
// Reads the 3 magnetometer channels and stores them in vector m
void readMag(void)
{
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(0x03);//LSM303_OUT_X_H_M
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDRESS, 6);
  while (Wire.available() < 6);
  byte xhm = Wire.read();
  byte xlm = Wire.read();
  byte  yhm = Wire.read();
  byte  ylm = Wire.read();
  byte  zhm = Wire.read();
  byte  zlm = Wire.read();

  MagX = (xhm << 8 | xlm);
  MagY = (yhm << 8 | ylm);
  MagZ = (zhm << 8 | zlm);
}

// myisr -- interrupt
void myisr1()  
{
  unsigned long now1 = micros();
  int val1 = digitalRead(pin1); 
  if(val1 == HIGH) // ascending edge, just save off start time
    start1 = now1;
  else // val == LOW, descending edge, compute pulse width 
   width2= now1 - start1;
   width2 = constrain(width2, 980, 2000);
}  
void myisr2()  
{
  unsigned long now2 = micros();
  int val2 = digitalRead(pin2);  
  if(val2 == HIGH)
    start2 = now2;
  else
   width1= now2 - start2;
   width1 = constrain(width1, 980, 2000);
} 
void myisr3()  
{
  unsigned long now3 = micros();
  int val3 = digitalRead(pin3); 
  if(val3 == HIGH) 
    start3 = now3;
  else
   width3= now3 - start3;
   width3 = constrain(width3, 980, 2000);
}
void myisr4()  
{
  unsigned long now4 = micros();
  int val4 = digitalRead(pin4);
  if(val4 == HIGH)
    start4 = now4;
  else
   width4= now4 - start4;
   width4 = constrain(width4, 980, 2000);
}
ISR(PCINT1_vect)
{
  unsigned long now5 = micros();
  int val5 = digitalRead(pin5); 
  if(val5 == HIGH) // ascending edge, just save off start time
    start5 = now5;
  else // val == LOW, descending edge, compute pulse width 
   width5= now5 - start5;
   width5 = constrain(width5, 980, 2000);
}
void commandAllMotors(int motorCommand) {
   for(int j = 0 ; j <= 50 ; j++)
 {
   motor1.writeMicroseconds(motorCommand); //FRONTL
   motor2.writeMicroseconds(motorCommand);//FRONTR
   motor3.writeMicroseconds(motorCommand); //REARL
   motor4.writeMicroseconds(motorCommand);//REARR
   delay(10);
  }  
}
void setup() {
  //attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds
  //default min is 544, max is 2400
  motor1.attach(5);
  motor2.attach(6);
  motor3.attach(7);
  motor4.attach(8);
   for(int j = 0 ; j <= 50 ; j++)
 {
   motor1.writeMicroseconds(1000); 
   motor2.writeMicroseconds(1000);
   motor3.writeMicroseconds(1000); 
   motor4.writeMicroseconds(1000);
   delay(20);
  }
  
  Serial.begin(115200);
  pinMode(12, OUTPUT);//LED
  digitalWrite(12, LOW);
  pinMode(2, INPUT);//INT4
  pinMode(3, INPUT);//INT5
  pinMode(19, INPUT);//INT2
  pinMode(18, INPUT);//INT3
  pinMode(15, INPUT);//PCINT9
  
  // pin change interrupt enables		
  PCICR |= (1 << PCIE1);// PCINT15:8
  // pin change masks
  PCMSK1 |= (1 << PCINT9);		// PIN 15
  //LOW to trigger the interrupt whenever the pin is low,
  //CHANGE to trigger the interrupt whenever the pin changes value
  //RISING to trigger when the pin goes from low to high,
  //FALLING for when the pin goes from high to low.
  attachInterrupt(intrnum1, myisr1, CHANGE);  
  attachInterrupt(intrnum2, myisr2, CHANGE);   
  attachInterrupt(intrnum3, myisr3, CHANGE); 
  attachInterrupt(intrnum4, myisr4, CHANGE); 
  
  Wire.begin();
  GyroInit();
  AccelerometerInit();
   for (int j = 0 ; j <= 100 ; j++) //Initialization sensor
  {
    Gyroread();
    readAcc();
    float R2 = sqrt(pow(MagX,2)+pow(MagY,2));
    float accXangle = atan2(sqrt(pow(AccY,2)+pow(AccZ,2)),AccX)*RAD_TO_DEG - 90;//6.8
    float accYangle = atan2(sqrt(pow(AccX,2)+pow(AccZ,2)),AccY)*RAD_TO_DEG*-1 + 90;//1.5
    
   gyroxtrim += GyroX;
   gyroytrim += GyroY;
   gyroztrim += GyroZ;
   xtrim += accXangle;
   ytrim += accYangle;
   ztrim += AccZ;
   delay(10);
   }
 
   gyroxtrim = gyroxtrim/100.0;
   gyroytrim = gyroytrim/100.0;
   gyroztrim = gyroztrim/100.0;
   xtrim = xtrim/100.0;
   ytrim = ytrim/100.0;
   ztrim = ztrim/100.0;
   Serial.print("calibrate sensor trim");Serial.print("\t ");
   Serial.print(xtrim);Serial.print("\t ");
   Serial.print(ytrim);Serial.print("\t ");
   Serial.print(ztrim);Serial.print("\t ");
   Serial.print(gyroxtrim);Serial.print("\t ");
   Serial.print(gyroytrim);Serial.print("\t ");
   Serial.println(gyroztrim);
   
   gyroxtrim = 0.25;//0.25
   gyroytrim = -0.85;//-0.85
   gyroztrim = -0.37;//-0.37
   xtrim = -5.3;//-2.83  -3.3
   ytrim = 1.7;//-0.44   1.7
   ztrim = 9.83;//9.83
   
  timer = millis();//start timing
}

void loop() {
  digitalWrite(12, LOW);
  float ro1;
  float pi1;
  float ro2;
  float pi2;
  float ya2;
  float err_roll;
  float err_roll_rate;
  float control_roll;
  float err_pitch;
  float err_pitch_rate;
  float control_pitch;
  float err_yaw_rate;
  float control_yaw;
  
  dtime = millis()-timer;
  if(dtime >= 10)////roop run = 10 ms = 100 Hz
  {
    timer = millis();//timing
    dt = dtime*0.001;
    Gyroread();
    readAcc();
    //readMag();
    
   //remote low pass filter
  //RC = 1/2*pi*fc  //a = dt / (RC + dt) ==(20Hz = 0.55)  (10Hz = 0.38)
  roll = roll + ((width2 - roll - 1500.0)*0.35); //1.941  0.25
  pitch = pitch + ((width1 - pitch - 1500.0)*0.35);
  thro = thro + ((width3 - thro)*0.35);
  yaw = yaw + ((width4 - yaw - 1500.0)*0.35);
  aux1 = aux1 + ((width5 - aux1)*0.35);//low pass filter
  
  float gyroXrate = GyroX - gyroxtrim;//-0.0091
  float gyroYrate = GyroY - gyroytrim;// 70 mdps/digit; 1 dps = 0.07 //FS = 250 dps 8.75
  float gyroZrate = GyroZ - gyroztrim;
  
  //AN3182 Application note Tilt measurement using a low-g 3-axis accelerometer
  float R2 = sqrt(pow(MagX,2)+pow(MagY,2));
  float accXangle = atan2(sqrt(pow(AccY,2)+pow(AccZ,2)),AccX)*RAD_TO_DEG - 90 - xtrim;//6.8
  float accYangle = atan2(sqrt(pow(AccX,2)+pow(AccZ,2)),AccY)*RAD_TO_DEG*-1 + 90 - ytrim;//1.5
  float accZangle = (acos(MagY/R2)*RAD_TO_DEG);
  
  //Gyro Dead Band
  float gdb = 0.1;
  if(gyroXrate > -gdb && gyroXrate < gdb)
    {
     gyroXrate = 0.0; 
    }
    if(gyroYrate > -gdb && gyroYrate < gdb)
    {
     gyroYrate = 0.0; 
    }
     if(gyroZrate > -gdb && gyroZrate < gdb)
    {
     gyroZrate = 0.0; 
    }
    
     //Complementary filter a = t/(t+dt) t=0.1
  float a=0.995;//0.99
  xAngle = (a*(xAngle + gyroXrate*dt)) + ((1-a)*accXangle);
  yAngle = (a*(yAngle + gyroYrate*dt)) + ((1-a)*accYangle);
  //zAngle = (a*(zAngle + gyroZrate*dt)) + ((1-a)*accZangle*mz); 
  xAngle = constrain(xAngle, -45, 45);
  yAngle = constrain(yAngle, -45, 45);
  zAngle = constrain(zAngle, -180, 180); 
  
   //โหมด Rate Acrobatic Mode  
    //ค่าความเร็วจากรีโมท
    ro2 = (roll*0.452);//0.452  = 190 degree/s
    pi2 = (pitch*-0.452);//-0.452 190 degree/s
    ya2 = (yaw*-0.551);  //-0.551 230 degree/s
     //ค่ามุมจากรีโมท
    ro1 = (roll*0.08); // 0.071 roll   +ofset angel
    pi1 = (pitch*-0.08); // 0.071 pitch
    
    if (aux1 > 1600)//Mode Flip 3D  exp(6.5) = 665 degree/s
    {
      
      ro2 = roll*0.5*exp(abs(roll*0.0003));//0.312    0.6=285 degree/s
      pi2 = pitch*-0.5*exp(abs(pitch*0.0003));//-0.312 
      ya2 = yaw*-0.5*exp(abs(yaw*-0.0003));  //-0.351 
    }
    //RC Dead Band
    float rdb = 10.0;
    if(ro2 > -rdb && ro2 < rdb)
    {
     ro1 = 0.0;  
     ro2 = 0.0; 
    }
    if(pi2 > -rdb && pi2 < rdb)
    {
     pi1 = 0.0; 
     pi2 = 0.0; 
    }
     if(ya2 > -rdb && ya2 < rdb)
    {
     ya2 = 0.0; 
    }
    
    if (aux1 < 1400) { //Mode Stable
    // ROLL CONTROL   
    err_roll = (ro1 - xAngle)*1.15;//
    err_roll_rate = ro2 - gyroXrate;
    roll_I2 = roll_I2 + (KI_RATE_ROLL*(err_roll_rate + err_roll)*dt); //ki =2.37
    roll_I2 = constrain(roll_I2, -100, 100);
    roll_D2 = (tar*roll_D2/(tar+dt)) + (KD_RATE_ROLL*(err_roll_rate-err_roll_old2)/(tar+dt));//0.75
    err_roll_old2 = err_roll_rate;
    
    control_roll = (err_roll_rate*KP_RATE_ROLL) + roll_I2 + roll_D2; 
    
    // PITCH CONTROL
    err_pitch = (pi1 - yAngle)*1.15;//
    err_pitch_rate = pi2 - gyroYrate;
    pitch_I2 = pitch_I2 + (KI_RATE_PITCH*(err_pitch_rate + err_pitch)*dt); //ki =2.37
    pitch_I2 = constrain(pitch_I2, -100, 100); 
    pitch_D2 = (tar*pitch_D2/(tar+dt)) + (KD_RATE_PITCH*(err_pitch_rate-err_pitch_old2)/(tar+dt));//0.75
    err_pitch_old2 = err_pitch_rate;
    
    control_pitch = (err_pitch_rate*KP_RATE_PITCH) + pitch_I2 + pitch_D2;
    
    // YAW CONTROL
    //err_yaw = (ya2 - gyroZrate);
    err_yaw_rate = (ya2 - gyroZrate);
    yaw_I = yaw_I + KI_RATE_YAW*err_yaw_rate*dt;
    yaw_I = constrain(yaw_I, -100, 100);
    yaw_D = (tar*yaw_D/(tar+dt)) + (KD_RATE_YAW*(err_yaw_rate-err_yaw_old)/(tar+dt));
    err_yaw_old = err_yaw_rate;
    
    control_yaw = KP_RATE_YAW*err_yaw_rate + yaw_D + yaw_I;// KD_QUAD_YAW*gyroZrate
    }
    
    if (aux1 > 1400 && aux1 < 1600) {//Mode Manual
    //โหมด Rate Acrobatic Mode 
    // ROLL CONTROL    
    err_roll_rate = ro2 - gyroXrate;
    roll_I2 = roll_I2 + (KI_RATE_ROLL*err_roll_rate*dt); //ki =2.37
    roll_I2 = constrain(roll_I2, -100, 100);
    roll_D2 = (tar*roll_D2/(tar+dt)) + (KD_RATE_ROLL*(err_roll_rate-err_roll_old2)/(tar+dt));//0.75
    err_roll_old2 = err_roll_rate;
    
    control_roll = (err_roll_rate*KP_RATE_ROLL) + roll_I2 + roll_D2; 
    
    // PITCH CONTROL
    err_pitch_rate = pi2 - gyroYrate;
    pitch_I2 = pitch_I2 + (KI_RATE_PITCH*err_pitch_rate*dt); //ki =2.37
    pitch_I2 = constrain(pitch_I2, -100, 100); 
    pitch_D2 = (tar*pitch_D2/(tar+dt)) + (KD_RATE_PITCH*(err_pitch_rate-err_pitch_old2)/(tar+dt));//0.75
    err_pitch_old2 = err_pitch_rate;
    
    control_pitch = (err_pitch_rate*KP_RATE_PITCH) + pitch_I2 + pitch_D2;
    
    // YAW CONTROL
    //err_yaw = (ya2 - gyroZrate);
    err_yaw_rate = (ya2 - gyroZrate);
    yaw_I = yaw_I + KI_RATE_YAW*err_yaw_rate*dt;
    yaw_I = constrain(yaw_I, -100, 100);
    yaw_D = (tar*yaw_D/(tar+dt)) + (KD_RATE_YAW*(err_yaw_rate-err_yaw_old)/(tar+dt));
    err_yaw_old = err_yaw_rate;
    
    control_yaw = KP_RATE_YAW*err_yaw_rate + yaw_D + yaw_I;// KD_QUAD_YAW*gyroZrate
    }
    if (aux1 > 1600) {//Mode Flip 3D
    
    // ROLL CONTROL    
    err_roll_rate = ro2 - gyroXrate;
    roll_I2 = roll_I2 + (KI_RATE_ROLL*(0.5*(err_roll_rate - err_roll_old2)*dt + (err_roll_old2*dt))); //new
    roll_I2 = constrain(roll_I2, -120, 120);
    roll_D2 = (tar*roll_D2/(tar+dt)) + (KD_RATE_ROLL*(err_roll_rate-err_roll_old2)/(tar+dt));//0.75
    err_roll_old2 = err_roll_rate;
    
    control_roll = (err_roll_rate*KP_RATE_ROLL) + roll_I2 + roll_D2; 
    
    // PITCH CONTROL
    err_pitch_rate = pi2 - gyroYrate;
    pitch_I2 = pitch_I2 + (KI_RATE_PITCH*(0.5*(err_pitch_rate - err_pitch_old2)*dt + (err_pitch_old2*dt))); //new
    pitch_I2 = constrain(pitch_I2, -120, 120); 
    pitch_D2 = (tar*pitch_D2/(tar+dt)) + (KD_RATE_PITCH*(err_pitch_rate-err_pitch_old2)/(tar+dt));
    err_pitch_old2 = err_pitch_rate;
    
    control_pitch = (err_pitch_rate*KP_RATE_PITCH) + pitch_I2 + pitch_D2;
    
    // YAW CONTROL
    //err_yaw = (ya2 - gyroZrate);
    err_yaw_rate = (ya2 - gyroZrate);
    yaw_I = yaw_I + (KI_RATE_YAW*(0.5*(err_yaw_rate - err_yaw_old)*dt + (err_yaw_old*dt))); //new
    yaw_I = constrain(yaw_I, -120, 120);
    yaw_D = (tar*yaw_D/(tar+dt)) + (KD_RATE_YAW*(err_yaw_rate-err_yaw_old)/(tar+dt));
    err_yaw_old = err_yaw_rate;
    
    control_yaw = KP_RATE_YAW*err_yaw_rate + yaw_D + yaw_I;// KD_QUAD_YAW*gyroZrate
    }
  int u1 = 1000;      // R
  int u2 = 1000;  // L
  int u3 = 1000;      // B
  int u4 = 1000;      // F

  //RUDDER  L to On  and RUDDER  R to Off
  if (thro < 1100) {
    if (yaw > 400 && armed == 1) {
      armed = 0;
      commandAllMotors(MINCOMMAND);
      digitalWrite(12, LOW);
    }
    if (yaw < -400 && armed == 0) {
      armed = 1;
      digitalWrite(12, HIGH);
      commandAllMotors(MINCOMMAND + 110);
      delay(500);
      commandAllMotors(MINCOMMAND);
    }
  }   
  if (armed == 0) {
    u1 = MINCOMMAND;
    u2 = MINCOMMAND;
    u3 = MINCOMMAND;
    u4 = MINCOMMAND;  
  }  
    if (armed == 1) {
    // Calculate motor commands
    u1 = thro + control_pitch + control_roll + control_yaw;//motor_FRONTL
    u2 = thro + control_pitch - control_roll - control_yaw;//motor_FRONTR
    u3 = thro - control_pitch + control_roll - control_yaw;//motor_REARL
    u4 = thro - control_pitch - control_roll + control_yaw;//motor_REARR      
  }
  if (thro < 1100) {
    u1 = 1005;
    u2 = 1005;
    u3 = 1005;
    u4 = 1005;   
    //roll_I1=0.0;
    //pitch_I1=0.0;
    roll_I2=0.0;
    pitch_I2=0.0;
    yaw_I=0.0;
  }
  u1 = constrain(u1, 1000, 1850);                 //pin D5  roll R   speed L (counter clockwise)
  u2 = constrain(u2, 1000, 1850);                 //pin D6  roll L   speed L (counter clockwise)
  u3 = constrain(u3, 1000, 1850);                 //pin D7  Pitch B  speed R (clockwise)
  u4 = constrain(u4, 1000, 1850);                 //pin D8  Pitch F  speed R (clockwise)
  motor1.writeMicroseconds(u1); 
  motor2.writeMicroseconds(u2);
  motor3.writeMicroseconds(u3); 
  motor4.writeMicroseconds(u4);
 
  i = i + 1;
    if(i >= 10)
      {
        i = 0;

   //Serial.print(thro);Serial.print("\t");
   //Serial.print(roll);Serial.print("\t");
   //Serial.print(pitch);Serial.print("\t");
   //Serial.print(yaw);Serial.print("\t");
   //Serial.print(aux1);Serial.print("\t");
   
  //Serial.print(u1);Serial.print("\t");
  //Serial.print(u2);Serial.print("\t");
  //Serial.print(u3);Serial.print("\t");
  //Serial.print(u4);Serial.print("\t");
  
    Serial.print(ro2);Serial.print("\t");
    Serial.print(pi2);Serial.print("\t");
    Serial.print(ya2);Serial.print("\t");
    
//Serial.print(control_roll);Serial.print("\t");
//Serial.print(control_pitch);Serial.print("\t");
//Serial.print(control_yaw); Serial.print("\t");
  
  
    //Serial.print(AccX);Serial.print("\t");
    //Serial.print(AccY);Serial.print("\t");
    //Serial.print(AccZ);Serial.print("\t");
  
    //Serial.print(accXangle);Serial.print("\t");
    //Serial.print(accYangle);Serial.print("\t");
  
    //Serial.print(xAngle);Serial.print("\t");
    //Serial.print(yAngle);Serial.print("\t");
  //Serial.print(zAngle,2);Serial.print("\t");
  
  //Serial.print(c_magnetom_x,3);Serial.print("\t");
  //Serial.print(c_magnetom_y,3);Serial.print("\t");
  
  Serial.print(gyroXrate);Serial.print("\t");
  Serial.print(gyroYrate);Serial.print("\t");
  Serial.print(gyroZrate);Serial.print("\t");
  
  Serial.print(roll_I2);Serial.print("\t");
  Serial.print(pitch_I2);Serial.print("\t");
  
  Serial.println(dtime);//dtime  timer
       }
   }
  }
