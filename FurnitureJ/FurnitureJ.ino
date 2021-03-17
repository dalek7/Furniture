#include <Time.h>
#include "Wire.h"

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"

//#include <Adafruit_MPL115A2.h>
//Adafruit_MPL115A2 mpl115a2;

#define highByte2(w) ((w) >> 8)
#define lowByte2(w) ((w) & 0xff)

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;
unsigned long ddt;

uint8_t buffer_a[6];
uint8_t buffer_g[6];
uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;

const int buttonPin = 2;     // the number of the pushbutton pin

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define LED_PIN_EXT 7 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

uint8_t dataPacket[24];

#define TO_PROCESSING
int timer1_counter;
void setup()
{

 
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_EXT, OUTPUT);

  digitalWrite(LED_PIN_EXT, HIGH);
  delay(500);
  digitalWrite(LED_PIN_EXT, LOW);
  delay(500);
  //mpl115a2.begin();
  
  for (int ii = 0; ii < 24; ii++)
  {
    dataPacket[ii] = 0;
  }

  dataPacket[0] = '$';
  dataPacket[1] = 0x02;

  dataPacket[20] = 0x00;
  dataPacket[21] = 0x00;
  dataPacket[22] = '\r';
  dataPacket[23] = '\n';

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  //Serial.println("Initializing I2C devices...");
  accelgyro.devAddr = 0x69;
  accelgyro.initialize();
  I2Cdev::writeByte(accelgyro.devAddr, MPU9250_RA_INT_PIN_CFG, 0x02); //set i2c bypass enable pin to true to access magnetometer
  delay(10);
  I2Cdev::writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);

  //    pinMode(INTERRUPT_PIN, INPUT);
  //    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DataReady, RISING);

  // verify connection
  //Serial.println("Testing device connections...");
  if(accelgyro.testConnection())
  {
     Serial.println("MPU9250 connection successful");
     digitalWrite(LED_PIN_EXT, HIGH);
  }
  else
  {
     Serial.println("MPU9250 connection failed");
     digitalWrite(LED_PIN_EXT, LOW);
  }
  

  // wait for ready
  /*
  Serial.println(F("\nSend any character to begin the program"));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
*/
  //  Mxyz_init_calibrated ();

  
}
/*
ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = timer1_counter;   // preload timer
  digitalWrite(LED_PIN_EXT, digitalRead(LED_PIN_EXT) ^ 1);
}
*/

volatile byte state = LOW;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void DataReady() {
  mpuInterrupt = true;
  state = !state;
  Serial.println(state);
}

void loop()
{

  
  
  
  /*
  int buttonState = digitalRead(buttonPin);
  digitalWrite(LED_PIN, buttonState);
  digitalWrite(LED_PIN_EXT, buttonState);
  */
  int sensor2 = analogRead(A0);
  int vh = 0;
  int vl = 0;
  if (sensor2  < 1024) 
  {
    vh = highByte2(sensor2);
    vl = lowByte2(sensor2);
    //vh = sensor2 / 512;
    //vl = sensor2 % 512;
  }

#ifdef TO_PROCESSING

  I2Cdev::readBytes(accelgyro.devAddr, MPU9250_RA_ACCEL_XOUT_H, 6, buffer_a);
  I2Cdev::readBytes(accelgyro.devAddr, MPU9250_RA_GYRO_XOUT_H, 6, buffer_g);

  //I2Cdev::readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  dataPacket[2] = buffer_a[0];
  dataPacket[3] = buffer_a[1];
  dataPacket[4] = buffer_a[2];
  dataPacket[5] = buffer_a[3];
  dataPacket[6] = buffer_a[4];
  dataPacket[7] = buffer_a[5];

  dataPacket[8] = buffer_g[0];
  dataPacket[9] = buffer_g[1];
  dataPacket[10] = buffer_g[2];
  dataPacket[11] = buffer_g[3];
  dataPacket[12] = buffer_g[4];
  dataPacket[13] = buffer_g[5];

  dataPacket[14] = 0x00;//buttonState;
  dataPacket[15] = vh;
  dataPacket[16] = vl;
  dataPacket[17] = 0x00;//buffer_m[3];
  dataPacket[18] = 0x00;//buffer_m[4];
  dataPacket[19] = 0x00;//buffer_m[5];
  
  Serial.write(dataPacket, 24);
  dataPacket[20]++;
#else
  getAccel_Data();
  getGyro_Data();
  //getCompassDate_calibrated();
/*
  Serial.print(Axyz[0]);
  Serial.print("\t");
  Serial.print(Axyz[1]);
  Serial.print("\t");
  Serial.print(Axyz[2]);
  Serial.print("\t");
  Serial.print(Gxyz[0]);
  Serial.print("\t");
  Serial.print(Gxyz[1]);
  Serial.print("\t");
  Serial.print(Gxyz[2]);
  Serial.print("\t");
  Serial.print(Mxyz[0]);
  Serial.print("\t");
  Serial.print(Mxyz[1]);
  Serial.print("\t");
  Serial.println(Mxyz[2]);
  */
#endif
}


void getHeading(void)
{
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
  if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1] / cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh) / PI;
  if (yh < 0)    tiltheading += 360;
}



void Mxyz_init_calibrated ()
{

  Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
  Serial.print("  ");
  Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
  Serial.print("  ");
  Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
  while (!Serial.find("ready"));
  Serial.println("  ");
  Serial.println("ready");
  Serial.println("Sample starting......");
  Serial.println("waiting ......");

  get_calibration_Data ();

  Serial.println("     ");
  Serial.println("compass calibration parameter ");
  Serial.print(mx_centre);
  Serial.print("     ");
  Serial.print(my_centre);
  Serial.print("     ");
  Serial.println(mz_centre);
  Serial.println("    ");
}


void get_calibration_Data ()
{
  for (int i = 0; i < sample_num_mdate; i++)
  {
    get_one_sample_date_mxyz();

    if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
    if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
    if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

    if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
    if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
    if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

  }

  mx_max = mx_sample[1];
  my_max = my_sample[1];
  mz_max = mz_sample[1];

  mx_min = mx_sample[0];
  my_min = my_sample[0];
  mz_min = mz_sample[0];

  mx_centre = (mx_max + mx_min) / 2;
  my_centre = (my_max + my_min) / 2;
  mz_centre = (mz_max + mz_min) / 2;

}

void get_one_sample_date_mxyz()
{
  getCompass_Data();
  mx_sample[2] = Mxyz[0];
  my_sample[2] = Mxyz[1];
  mz_sample[2] = Mxyz[2];
}


void getAccel_Data(void)
{
  //    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  accelgyro.getAcceleration(&ax, &ay, &az);

  buffer_a[1] = ax & 0x00FF;
  buffer_a[0] = (ax >> 8) & 0x00FF;

  buffer_a[3] = ay & 0x00FF;
  buffer_a[2] = (ay >> 8) & 0x00FF;

  buffer_a[5] = az & 0x00FF;
  buffer_a[4] = (az >> 8) & 0x00FF;

  Axyz[0] = (double) (ax) * 2 / 32768;
  Axyz[1] = (double) (ay) * 2 / 32768;
  Axyz[2] = (double) (az) * 2 / 32768;
}


void getGyro_Data(void)
{
  //    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  accelgyro.getRotation(&gx, &gy, &gz);

  buffer_g[1] = gx & 0x00FF;
  buffer_g[0] = (gx >> 8) & 0x00FF;

  buffer_g[3] = gy & 0x00FF;
  buffer_g[2] = (gy >> 8) & 0x00FF;

  buffer_g[5] = gz & 0x00FF;
  buffer_g[4] = (gz >> 8) & 0x00FF;

  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
  I2Cdev::writeByte(accelgyro.devAddr, MPU9250_RA_INT_PIN_CFG, 0x02); //set i2c bypass enable pin to true to access magnetometer
  delay(10);
  I2Cdev::writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2Cdev::readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

  Mxyz[0] = (double) mx * 4800 / 16384;
  Mxyz[1] = (double) my * 4800 / 16384;
  Mxyz[2] = (double) mz * 4800 / 16384;
}

void getCompassDate_calibrated ()
{
  getCompass_Data();
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;
}
