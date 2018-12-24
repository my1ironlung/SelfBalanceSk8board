
#include <Wire.h> // I2C library, gyroscope

// Accelerometer ADXL345
#define ACC (0x53)    //ADXL345 ACC address
#define A_TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)


// Gyroscope ITG3200 
#define GYRO 0x68 // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E

#define G_TO_READ 8 // 2 bytes for each axis x, y, z


#include <SoftwareSerial.h>


//******************** IMPORTANT ***************************
//Set dip switches on the Sabertooth for simplified serial and 9600 Buadrate. Diagram of this on my Instructables page//
//******************** IMPORTANT ***************************


//Digital pin 13 is serial transmit pin to sabertooth
#define SABER_TX_PIN  13
//Not used but still initialised, Digital pin 12 is serial receive from Sabertooth
#define SABER_RX_PIN  12

//set baudrate to match sabertooth dip settings
#define SABER_BAUDRATE  9600

//simplifierd serial limits for each motor
#define SABER_MOTOR1_FULL_FORWARD 127
#define SABER_MOTOR1_FULL_REVERSE 1

#define SABER_MOTOR2_FULL_FORWARD 255
#define SABER_MOTOR2_FULL_REVERSE 128

//motor level to send when issuing full stop command
#define SABER_ALL_STOP  0


SoftwareSerial SaberSerial = SoftwareSerial (SABER_RX_PIN, SABER_TX_PIN );

void initSabertooth (void)  {
 //initialise software to communicate with sabertooth
pinMode ( SABER_TX_PIN, OUTPUT );


SaberSerial.begin( SABER_BAUDRATE );
//2 sec delay to let it settle
delay (2000);
SaberSerial.write((byte)0x0);   //kill motors when first switched on
}



//setup all variables. Terms may have strange names but this software has evolved from bits and bobs done by other segway clone builders
float level=0;


const int LED = 2; // LED connected to


float Steering;
float SteerValue;
float SteerCorrect;
float steersum;
int Steer = 0;
float SteerFactor= .15;
float gainMax=.45;


float x_accdeg;


float gangleratedeg;
float gangleratedeg2;

int adc1;
int adc4;
int adc5;


float gangleraterads;
float gyroscalingfactor = 2.2;


int k0; //for steering
float k1;
int k2;
int k3;
int k4;


//SAFET vARS
int safetyTime;
boolean safetyKill=false;
int safetyCounter;

float lambda=.1;
float beta=.1;
float tau;
float tauPast1;

float overallgain; 

float gyroangledt;
float angle;
float anglerads;
float balance_torque;
float softstart;

float cur_speed;
//Need to know cycle time as gyro measures rate of turning. Needs to know time between each measurement
//so it can then work out angle it has turned through since the last measurement - so it can know angle of tilt from vertical.
float cycle_time;
float Balance_point;
float balancetrim;

int balleft;
int balright;



int i;
int j;
int tipstart;

signed char Motor1percent;
signed char Motor2percent;



//digital inputs
int deadmanbuttonPin = 9;  // deadman button is digital input pin 

int balancepointleftPin = 7; //if digital pin 7 is 5V then reduce balancepoint variable. Allows manual fine tune of the ideal target balance point
int balancepointrightPin = 6; //if digital pin 6 is 5V then increase balancepoint variable. Allows manual fine tune of the ideal target balance point
int steeringvariableleftPin = 5; //digital pin5 Used to steer
int steeringvariablerightPin = 4; //digital pin 4 Used to steer the other way.

//digital outputs


//analog inputs
int nunchuckPin=0;



// offsets are chip specific. 

//int g_offx = 34;
//int g_offy = 20;
//int g_offz = 93;
int g_offx;
int g_offy;
int g_offz;

float(temp);

char str[512]; 


boolean firstSample = true;

float RwAcc[3];  //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float RwAccPast1[3]; //past projections for averaging
float RwAccPast2[3];
float RwAccEst[3];
float RwAccEstPast1[3];
float Gyro_ds[3];  //Gyro readings         
float RwGyro[3];        //Rw obtained from last estimated value and gyro movement
float Awz[2];           //angles between projection of R on XZ/YZ plane and Z axis (deg)
float RwEst[3];

int lastTime = 0;
int interval = 0;
float wGyro = 10.0;

void initAcc() {
 //Turning on the ADXL345
 writeTo(ACC, 0x2D, 0);      
 writeTo(ACC, 0x2D, 16);
 writeTo(ACC, 0x2D, 8);
 //by default the device is in +-2g range reading
}

void getAccelerometerData(int * result) {
 /*
  RwAccPast2[0]=RwAccPast1[0];
 RwAccPast2[1]=RwAccPast1[1];
 RwAccPast2[2]=RwAccPast1[2];
 //RwAccPast2[3]=RwAccPast1[3];
 */
  RwAccPast1[0]=RwAcc[0];
 RwAccPast1[1]=RwAcc[1];
 RwAccPast1[2]=RwAcc[2];
 
 
 RwAccEstPast1[0]=RwAccEst[0];
 RwAccEstPast1[1]=RwAccEst[1];
 RwAccEstPast1[2]=RwAccEst[2];


 int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
 byte buff[A_TO_READ];

 readFrom(ACC, regAddress, A_TO_READ, buff); //read the acceleration data from the ADXL345

 //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
 //thus we are converting both bytes in to one int
 result[0] = (((int)buff[1]) << 8) | buff[0];   
 result[1] = (((int)buff[3])<< 8) | buff[2];
 result[2] = (((int)buff[5]) << 8) | buff[4];
}

void rawAccToG(int * raw, float * RwAcc) {
 RwAcc[0] = ((float) raw[0]) / 256.0;
 RwAcc[1] = ((float) raw[1]) / 256.0;
 RwAcc[2] = ((float) raw[2]) / 256.0;

/*
 //Average the raw ACC from past reading by 2
RwAcc[0]= (RwAcc[0]+ RwAccPast1[0])/2;
RwAcc[1]= (RwAcc[1]+ RwAccPast1[1])/2;
RwAcc[2]= (RwAcc[2]+ RwAccPast1[2])/2;
*/



if (firstSample){
for (i=0; i<=2; i++){
  
   RwAccEst[i]=RwAcc[i];
   tau=RwAccEst[i];
   
 }

}
 
else 
{
  for (i=0; i<=2; i++){
    
    if (abs(RwAcc[i]>2)){
    
      if (RwAcc<0){
      RwAcc[i]=-2.0;
        }
      else{RwAcc[i]=2.0;
       }
    }
    else{
    //Schlain filter1 literature found here: http://www.itl.nist.gov/div898/handbook/pmc/section4/pmc432.htm
    // RwAccEst[i]= lambda*RwAcc[i]+ ((1-lambda)*RwAccEstPast1[i]);
     
     //schlain filter 2 (double exponential filter, literature can be found here: http://www.itl.nist.gov/div898/handbook/pmc/section4/pmc433.htm )
     //the larger the values of lambda and beta, the less filtering there will be and more noise.
     RwAccEst[i]= lambda*RwAcc[i]+ ((1-lambda)*(RwAccEstPast1[i]+tauPast1));
     tau=(beta*(RwAccEst[i]-RwAccEstPast1[i]))+((1-beta)*tauPast1);
    }
  }
}
tauPast1=tau;

//Serial.println(RwAcc[2]);

//Serial.println(RwAcc[0]);
//Serial.print("         ");
//Serial.println(RwAccEst[0]);

/*
//average by past 3
RwAcc[0]= (RwAcc[0]+ RwAccPast1[0]+ RwAccPast2[0])/3;
RwAcc[1]= (RwAcc[1]+ RwAccPast1[1]+ RwAccPast2[1])/3;
RwAcc[2]= (RwAcc[2]+ RwAccPast1[2]+ RwAccPast2[2])/3;


Serial.println(RwAcc[0]);
Serial.print("         ");
Serial.print(RwAcc[1]);
Serial.print("         ");
Serial.print(RwAcc[2]);
Serial.print("         ");
Serial.print(RwAcc[3]);


Serial.println(RwAccEst[0]);
Serial.print("         ");
Serial.print(RwAccEst[1]);
Serial.print("         ");
Serial.print(RwAccEst[2]);
Serial.print("         ");
Serial.print(RwAccEst[3]);
*/
}

//initializes the gyroscope
void initGyro()
{
 /*****************************************
 * ITG 3200
 * power management set to:
 * clock select = internal oscillator
 *     no reset, no sleep mode
 *   no standby mode
 * sample rate to = 125Hz
 * parameter to +/- 2000 degrees/sec
 * low pass filter = 5Hz
 * no interrupt
 ******************************************/
 writeTo(GYRO, G_PWR_MGM, 0x00);
 writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
 writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
 writeTo(GYRO, G_INT_CFG, 0x00);
}


void getGyroscopeData(int * result)
{
 /**************************************
 Gyro ITG-3200 I2C
 registers:
 temp MSB = 1B, temp LSB = 1C
 x axis MSB = 1D, x axis LSB = 1E
 y axis MSB = 1F, y axis LSB = 20
 z axis MSB = 21, z axis LSB = 22
 *************************************/

 int regAddress = 0x1B;
 int temp, x, y, z;
 byte buff[G_TO_READ];



 readFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200

 result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
 result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
 result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
 result[3] = (buff[0] << 8) | buff[1]; // temperature 
}


// convert raw readings to degrees/sec
void rawGyroToDegsec(int * raw, float * gyro_ds) {
 gyro_ds[0] = ((float) raw[0]) / 14.375;
 gyro_ds[1] = ((float) raw[1]) / 14.375;
 gyro_ds[2] = ((float) raw[2]) / 14.375;
}


void normalize3DVec(float * vector) {
 float R;
 R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
 vector[0] /= R;
 vector[1] /= R;  
 vector[2] /= R;  
}


float squared(float x){
 return x*x;
}


void getInclination() {
 int w = 0;
 float tmpf = 0.0;
 int currentTime, signRzGyro;


 currentTime = millis();
 interval = currentTime - lastTime;
 lastTime = currentTime;

 if (firstSample) { // the NaN check is used to wait for good data from the Arduino
   for(w=0;w<=2;w++) {
     RwEst[w] = RwAcc[w];    //initialize with accelerometer readings
   }
 }
 else{
   //evaluate RwGyro vector
   if(abs(RwEst[2]) < 0.1) {
     //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
     //in this case skip the gyro data and just use previous estimate
     for(w=0;w<=2;w++) {
       RwGyro[w] = RwEst[w];
     }
   }
   else {
     //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
     for(w=0;w<=1;w++){
       tmpf = Gyro_ds[w];                        //get current gyro rate in deg/s
       tmpf *= interval / 1000.0f;                     //get angle change in deg
       Awz[w] = atan2(RwEst[w],RwEst[2]) * 180 / PI;   //get angle and convert to degrees 
       Awz[w] += tmpf;             //get updated angle according to gyro movement
     }

     //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
     //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
     signRzGyro = ( cos(Awz[0] * PI / 180) >=0 ) ? 1 : -1;

     //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
     for(w=0;w<=1;w++){
       RwGyro[0] = sin(Awz[0] * PI / 180);
       RwGyro[0] /= sqrt( 1 + squared(cos(Awz[0] * PI / 180)) * squared(tan(Awz[1] * PI / 180)) );
       RwGyro[1] = sin(Awz[1] * PI / 180);
       RwGyro[1] /= sqrt( 1 + squared(cos(Awz[1] * PI / 180)) * squared(tan(Awz[0] * PI / 180)) );        
     }
     RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
   }

   //combine Accelerometer and gyro readings
   for(w=0;w<=2;w++) RwEst[w] = (RwAccEst[w] + wGyro * RwGyro[w]) / (1 + wGyro);
   /*
   Serial.print("RwEst0="); //acc+gyro combined tilt angle
   Serial.print(RwEst[0]);  //acc+gyro combined tilt angle
   Serial.print("   GyroRateOfTiltingDeg/sec=");
   Serial.print(Gyro_ds[0]);
   Serial.print("   GyroRateOfTurningDeg/sec=");
   Serial.println(Gyro_ds[1]);
   */
   //Serial.print("  RwEst1=");
   //Serial.print(RwEst[1]);
   //Serial.print("  RwEst2=");
   //Serial.println(RwEst[2]);



   normalize3DVec(RwEst);
 }

 firstSample = false;
}



void setup()
{
 Serial.begin(9600); // HARD wired Serial feedback to PC for debugging in Wiring
 Wire.begin();
 initAcc();
 initGyro();
 initSabertooth( );

   //digital inputs
 pinMode(deadmanbuttonPin, INPUT);
 pinMode(balancepointleftPin, INPUT);
 pinMode(balancepointrightPin, INPUT);
 pinMode(steeringvariableleftPin, INPUT);
 pinMode(steeringvariablerightPin, INPUT);

   pinMode(LED, OUTPUT);

}





void loop()
{
   digitalWrite(LED, LOW);

 int counter = 0;
 float Gyro_ds0offset;
 float Gyro_ds1offset;
 float Gyro_ds2offset;
 //Serial.println("Keep machine still and tilted while sensors zero themselves, starting sample in 3 seconds");
 delay(3000);
 //Serial.println("Sampling now, taking 200 baseline samples");


 for(int n=0;n<=200;n++){
 if(!Serial.available()) {
   int acc[3];
   int gyro[4];


   getAccelerometerData(acc);
   rawAccToG(acc, RwAcc);
   normalize3DVec(RwAcc);

   getGyroscopeData(gyro);
   rawGyroToDegsec(gyro, Gyro_ds);

   getInclination();
   counter = counter + 1;
   Gyro_ds0offset = (float)Gyro_ds0offset + Gyro_ds[0];
   Gyro_ds1offset = (float)Gyro_ds1offset + Gyro_ds[1];

/*
   Serial.print("Gyro_ds0offset=");
   Serial.print(Gyro_ds0offset);
   Serial.print("  Gyro_ds1offset=");
   Serial.println(Gyro_ds1offset);
*/
                          }
                        } //end of for n = 0 to 200
     Gyro_ds0offset = (float)Gyro_ds0offset/counter;  //sets the offset
     Gyro_ds1offset = (float)Gyro_ds1offset/counter;  //sets the offset

//   Serial.print("XXXXXXXXXXXXXXXXXXXXXXXXX Gyro_ds0 averaged offset=");
  // Serial.println(Gyro_ds0offset);
  // Serial.print("XXXXXXXXXXXXXXXXXXXXXXXXX Gyro_ds1 averaged offset=");
  // Serial.println(Gyro_ds1offset);

     temp = (float)(g_offx - (Gyro_ds0offset * 14.375));
     g_offx = (int)temp;
     //Serial.print("g_offx=");
     //Serial.println(g_offx);
     temp = (float)(g_offy - (Gyro_ds1offset * 14.375));
     g_offy = (int)temp;
     //Serial.print("g_offy=");
     //Serial.println(g_offy);      


//XXXXXXXXXXXXXX TipStart XXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    //Serial.println("Now slowly bring it level to engage tipstart");

      digitalWrite(LED, HIGH);
 tipstart = 0;
 overallgain = 0;
 cur_speed = 0;
 level = 0;
 Steer = 0;
 balancetrim = 0;

 while (tipstart < 5) {

     if(!Serial.available()) {
     int acc[3];
     int gyro[4];


     getAccelerometerData(acc);
     rawAccToG(acc, RwAcc);
     normalize3DVec(RwAcc);

     getGyroscopeData(gyro);
     rawGyroToDegsec(gyro, Gyro_ds);

     getInclination();

        // Serial.println("RwEst0="); //acc+gyro combined tilt angle
         //Serial.print(RwEst[0]);  //acc+gyro combined tilt angle  range -1g to +1g
     if (RwEst[0] < -0.1 || RwEst[0] > 0.1) {
             //*****ADJUST THESE LIMITS TO SUIT YOUR BOARD SO TILTSTART KICKS IN WHERE YOU WANT IT TO*******

                                 tipstart = 0;
                                 overallgain = 0;
                                 cur_speed = 0;
                                 level = 0;
                                 Steer = 0;
                                 balancetrim = 0; 
                                                  }
                                                    else {
                    tipstart = 5;
       //             Serial.println("Tipstart activated");
                   }                                               


                          } //end of if serial available
            }//while tipstart < 5
//XXXXXXXXXXXXXXXX end of tipstart XXXXXXXXXXXXXXXXX

overallgain = 0.3; //softstart value. Gain will now rise to final of 0.5 at rate of 0.005 per program loop. 
//i.e. it will go from 0.3 to 0.5 over the first 4 seconds after tipstart has been activated
angle = 0;
cur_speed = 0;
Steering = 251;
SteerValue = 251;
balancetrim = 0;

//end of tiltstart code. If go beyond this point then machine is active
//main balance routine, just loops forever. Machine is just trying to stay level. You "trick" it into moving by tilting one end down
//works best if keep legs stiff so you are more rigid like a broom handle is if you are balancing it vertically on end of your finger
//if you are all wobbly, the board will go crazy trying to correct your own flexibility.
//NB: This is why a segway has to have vertical handlebar otherwise ankle joint flexibility in fore-aft direction would make it oscillate wildly.
//NB: This is why the handlebar-less version of Toyota Winglet still has a vertical section you jam between your knees.    


   while(1){

     if(!Serial.available()) {
     int acc[3];
     int gyro[4];


     getAccelerometerData(acc);
     rawAccToG(acc, RwAcc);
     normalize3DVec(RwAcc);

     getGyroscopeData(gyro);
     rawGyroToDegsec(gyro, Gyro_ds);

     getInclination();

         //Serial.print("RwEst0="); //acc+gyro combined tilt angle
         //Serial.print(RwEst[0]);  //acc+gyro combined tilt angle
         //Serial.print("   GyroRateOfTiltingDeg/sec=");
         //Serial.print(Gyro_ds[0]);
         //Serial.print("   GyroRateOfTurningDeg/sec=");
         //Serial.println(Gyro_ds[1]);


    //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX Insert PID and signals to motor HERE XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    sample_inputs(); //actually this is really "modify inputs from new IMU to fit in with previous balancing code"
    set_motor();
    safetyFirst();
   

    //serialOut_timing();//optional displays loop time on screen (first digit is time loop takes to run in millisec, 

  //second digit is final time for loop including the variable added delay to keep it at 100Hz

 //XXXXXXXXXXXXXXXXXXXX softstart function: board a bit squishy when you first bring it to balanced point, 
//then ride becomes firmer over next 4 seconds as value for overallgain increases from starting value of 0.3 to 0.5 we have here 
  if (overallgain < gainMax) {
      overallgain = (float)overallgain + 0.005;

                         }
  if (overallgain > gainMax) {overallgain = gainMax;}
//XXXXXXXXXXXXXXX end of softstart code XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX     

                          }//end of if serial available
            }//end of while(1)  This now loops forever

}//end of void loop             











void serialPrintFloatArr(float * arr, int length) {
 for(int i=0; i<length; i++) {
   serialFloatPrint(arr[i]);
   Serial.print(",");
 }
}


void serialFloatPrint(float f) {
 byte * b = (byte *) &f;
 Serial.print("f:");
 for(int i=0; i<4; i++) {

   byte b1 = (b[i] >> 4) & 0x0f;
   byte b2 = (b[i] & 0x0f);

   char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
   char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

   Serial.print(c1);
   Serial.print(c2);
 }
}


//---------------- Functions
//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}


//reads num bytes starting from address register on ACC in to buff array
void readFrom(int DEVICE, byte address, int num, byte buff[]) {
 Wire.beginTransmission(DEVICE); //start transmission to ACC 
 Wire.write(address);        //sends address to read from
 Wire.endTransmission(); //end transmission

 Wire.beginTransmission(DEVICE); //start transmission to ACC
 Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC

 int i = 0;
 while(Wire.available())    //ACC may send less than requested (abnormal)
 { 
   buff[i] = Wire.read(); // receive a byte
   i++;
 }
 Wire.endTransmission(); //end transmission
}





void sample_inputs()  {


 k2 = digitalRead(steeringvariableleftPin);
 k3 = digitalRead(steeringvariablerightPin);
 k4 = digitalRead(deadmanbuttonPin);
 //HARRY
 k0 = analogRead(nunchuckPin);

 balleft = digitalRead(balancepointleftPin);
 balright = digitalRead(balancepointrightPin);


 //XXXXXXXXXXXX remove all these when you want to add steering, balance adjust and active deadman
 balleft = 0; //temporarily disables this while you are trying to get it to balance
 balright = 0; //temporarily disables this while you are trying to get it to balance
 k2 = 0;//temporarily disables this while you are trying to get it to balance
 k3 = 0;//temporarily disables this while you are trying to get it to balance
 //k4 = 1;//temporarily ACTIVATES deadman 
 //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX



 if (balleft == 1) balancetrim = balancetrim - 0.04; //if pressing balance point adjust switch then slowly alter the balancetrim variable by 0.04 per loop of the program 
 //while you are pressing the switch
 if (balright == 1) balancetrim = balancetrim + 0.04; //same again in other direction
 if (balancetrim < -30) balancetrim = -30; //stops you going too far with this
 if (balancetrim > 30) balancetrim = 30; //stops you going too far the other way




//****COMMENT THIS STEERING SECTION OUT IF YOU DONT WANT ANY STEERING FUNCTIONS TO BEGIN WITH. JUST KEEP THE COMMAND: SteerValue=251; and add a command: SteerCorrect = 0;
//****COMMENT THIS STEERING SECTION OUT IF YOU DONT WANT ANY STEERING FUNCTIONS TO BEGIN WITH. JUST KEEP THE COMMAND: SteerValue=251; and add a command: SteerCorrect = 0;
//****COMMENT THIS STEERING SECTION OUT IF YOU DONT WANT ANY STEERING FUNCTIONS TO BEGIN WITH. JUST KEEP THE COMMAND: SteerValue=251; and add a command: SteerCorrect = 0; 
//****COMMENT THIS STEERING SECTION OUT IF YOU DONT WANT ANY STEERING FUNCTIONS TO BEGIN WITH. JUST KEEP THE COMMAND: SteerValue=251; and add a command: SteerCorrect = 0;

 gangleratedeg2 = Gyro_ds[1];
   // NO steering wanted. Use second gyro to maintain a (roughly) straight line heading (it will drift a bit).
 if (k0<=254 && k0 >= 246)  {



             SteerCorrect = 0; //blocks the direction stabiliser unless rate of turn exceeds -10 or +10 degrees per sec
         if (gangleratedeg2 > 10 || gangleratedeg2 < -10) {   //resists turning if turn rate exceeds 10deg per sec
                        SteerCorrect = (float) 0.4 * gangleratedeg2; //vary the 0.4 according to how much "resistance" to being nudged off course you want.
                        //a value called SteerCorrect is added to the steering value proportional to the rate of unwanted turning. It keeps getting
                        //larger if this condition is till being satisfied i.e. still turning >10deg per sec until the change has been resisted.
                        //can experiment with the value of 10. Try 5 deg per sec if you want - play around - this can probably be improved
                        //but if you try to turn it fast with your hand while balancing you will feel it resisting you so it does work
                        //also, when coming to a stop, one motor has a bit more friction than the other so as this motor stops first then as board
                        //comes to standstill it spins round and you can fall off. This is original reason I built in this feature.
                        //if motors have same friction you will not notice it so much.
                                                          }

             SteerValue = 0;

                             }



   else {                          
 //i.e. we DO want to steer


 //steer one way        SteerValue of 251 is straight ahead
if (k0 <= 245) {
       SteerValue=k0;     
      SteerValue = map(SteerValue, 60, 245, -100, 0);
 // Serial.print("left: ");
 // Serial.println(SteerValue);
             }
//change the 5 and -5 values if you want faster turn rates. Could use a potentiometer to control these values so would have proportional control of steering         

//steer the other way             
 if (k0 >= 255) {
           
   SteerValue=k0;     
      SteerValue = map(SteerValue, 255, 576, 0, 100);
   
  // Serial.print("right: ");
  //Serial.println(SteerValue);
             }   



   SteerCorrect = 0;

   }    


//Serial.println(k0);

 //*****END OF STEERING SECTION
 //*****END OF STEERING SECTION
 //*****END OF STEERING SECTION
 //*****END OF STEERING SECTION


/*

           //Serial.print("RwEst0="); //acc+gyro combined tilt angle
         Serial.print(RwEst[0]);  //acc+gyro combined tilt angle
         //Serial.print("   GyroRateOfTiltingDeg/sec=");
         Serial.print(Gyro_ds[0]);
         //Serial.print("   GyroRateOfTurningDeg/sec=");
         Serial.println(Gyro_ds[1]);
*/ 



 //ACCELEROMETER notes for the 5 d of f Sparfun IMU I have used in my Instructable:

//GYRO NOTES: 
//Low resolution gyro output: 2mV per degree per sec up to 500deg per sec. 5V = 1024 units on 0-1023 scale, 1Volt = 204.8 units on this scale. 
//2mV = 0.41 units = 1deg per sec
// Hi res gyro output pin(from the same gyro): 9.1mV per degree per sec up to 110deg per sec on hires input. 5V = 1024 units on 0-1023 scale, 1Volt = 204.8 units on this scale. 
//9.1mV = 1.86 units = 1 deg per sec


 gangleratedeg = Gyro_ds[0];

 if (gangleratedeg < -450) gangleratedeg = -450; //stops crazy values entering rest of the program
 if (gangleratedeg > 450) gangleratedeg = 450;


  //Key calculations. Gyro measures rate of tilt gangleratedeg in degrees. We know time since last measurement is cycle_time (5.5ms) so can work out much we have tipped over since last measurement
 //What is gyroscalingfactor variable? Strictly it should be 1. However if you tilt board/IMU to a fixed angle of say 10 degrees, with computer attached, then "angle" displayed on screen 
 //should be 10 (mainly due to the gyro data), if you hold it at 10 degrees, then accel will slowly fine tune the value. 
 //However, if you tilt it, and it reads 15 then slowly decreases back to 10, this means gyro is overreading, i.e. the "gyroscalingfactor" needs reducing slightly.
 //If you tilted it to 10 degree angle and readout gace angle of 7 which then slowly correcte to 10 with time, the gyro would be underrreading and the "gyroscalingfactor" 
 //needs to be increased slightly.



 gangleraterads = (float) gangleratedeg * 0.017453; //convert to radians - just a scaling issue from history
 angle = RwEst[0] * 57;  //rough approximation converts -1 to +1g to an angle in degrees


 anglerads = (float) angle * 0.017453; //converting to radians again a historic scaling issue from past software





 balance_torque = (float) (3.8  * anglerads) + (0.3 * gangleraterads); //power to motors (will be adjusted for each motor later to create any steering effects
 //balance torque is motor control variable we would use even if we just ahd one motor. It is what is required to make the thing balance only.
 //the values of 4.5 and 0.5 came from Trevor Blackwell's segway clone experiments and were derived by good old trial and error
 //I have also found them to be about right
 //We set the torque proportionally to the actual angle of tilt (anglerads), and also proportional to the RATE of tipping over (ganglerate rads)
 //the 4.5 and the 0.5 set the amount of each we use - play around with them if you want.
 //Much more on all this, PID controlo etc on my website

 cur_speed = (float) (cur_speed + (anglerads * 6 * interval * 0.001)) * 0.999;   //interval is in millisec e.g. 7, but here we want it in seconds so we multiply by 0.001 
 //So it means "if we are STILL tilted, speed up a bit" and it keeps accelerating as long as you hold it tilted.
 //You do NOT need this to just balance, but to go up a slight incline for example you would need it: if board hits incline and then stops - if you hold it
 //tilted for long eneough, it will eventually go up the slope (so long as motors powerfull enough and motor controller powerful enough)
 //Why the 0.999 value? I got this from the SegWii project code - thanks!
 //If you have built up a large cur_speed value and you tilt it back to come to a standstill, you will have to keep it tilted back even when you have come to rest
 //i.e. board will stop moving OK but will now not be level as you are tiliting it back other way to counteract this large cur_speed value that has built up.
 //The 0.999 means that if you bring board level after a long period tilted forwards, the cur_speed value magically decays away to nothing and your board
 //is now not only stationary but also level, very useful!


 //level = (float)(balance_torque + cur_speed) * overallgain;  
 level = (float)balance_torque * overallgain;  //You can omit cur speed term during testing while just getting it to initially balance if you want to
 //avoids confusion


}//end of sample inputs






void set_motor()   {
 unsigned char cSpeedVal_Motor1 = 0;
 unsigned char cSpeedVal_Motor2 = 0;

 level = level * 200; //changes it to a scale of about -100 to +100
 if (level < -100) {level = -100;}
 if (level > 100) {level = 100;}


 Steer = (float) SteerValue - SteerCorrect;  //at this point is on the 0-1023 scale 
 //SteerValue is either 512 for dead ahead or bigger/smaller if you are pressing steering switch left or right
 //SteerCorrect is the "adjustment" made by the second gyro that resists sudden turns if one wheel hits a small object for example.
 Steer *= SteerFactor;   //gets it down from 0-1023 (with 251 as the middle no-steer point) to -100 to +100 with 0 as the middle no-steer point on scale





//set motors using the simplified serial Sabertooth protocol (same for smaller 2 x 5 Watt Sabertooth by the way) 

Motor1percent = (signed char) level + Steer;
Motor2percent = (signed char) level - Steer;

if (Motor1percent > 100) Motor1percent = 100;
if (Motor1percent < -100) Motor1percent = -100;
if (Motor2percent > 100) Motor2percent = 100;
if (Motor2percent < -100) Motor2percent = -100;


//if not pressing deadman button on hand controller - cut everything
  if (k4 < 1) { 
   level = 0;
   Steer = 0;
   Motor1percent = 0;
   Motor2percent = 0;
              }
              
                if (safetyKill) { 
    level = 0;
    Steer = 0;
    Motor1percent = 0;
    Motor2percent = 0;
               }
 
 
//Serial.println(k4);

cSpeedVal_Motor1 = map (Motor1percent,
                        -100,
                        100,
                        SABER_MOTOR1_FULL_REVERSE,
                        SABER_MOTOR1_FULL_FORWARD);

cSpeedVal_Motor2 = map (Motor2percent,
                          -100,
                         100,
                        SABER_MOTOR2_FULL_REVERSE,
                        SABER_MOTOR2_FULL_FORWARD);

SaberSerial.write ((byte)cSpeedVal_Motor1);
SaberSerial.write ((byte)cSpeedVal_Motor2);

//Serial.println(level);

}







  void serialOut_timing(){
  static int skip=0;
  
  if (skip++==0) { //display every 500ms (at 100Hz)
    skip = 0;
   // Serial.print("Interval i.e.looptime");
    //Serial.println(interval);


      //XXXXXXXXXXXXXX put any variables you want printed to serial window in here XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  // Serial.print("gangleratedeg:");
   //Serial.print(gangleratedeg);

  // Serial.print("  angle:");
   //Serial.print(angle);
   //Serial.print("acc:  ");
   Serial.print(RwAcc[0]);
   Serial.print("|");
   Serial.println(RwEst[0]);
   //Serial.print("   Overallgain:");
   //Serial.println(overallgain);
   //Serial.print("   level:");
   //Serial.println(level);    



 
     //XXXXXXXXXXXXXX end of watch values in serial window XXXXXXXXXXXXXXXXXXXXXXXXXXX
                   }
                  
                           }
                           
                            void safetyFirst(){
          
       if (abs(level)>=100) {
            
             safetyCounter+=1;
             
             if (safetyCounter >=90){
                 safetyKill=true;
             }
       }
       
     else{
    // safetyCounter=0;
     //safetyKill=false;
     
     
     }
     //Serial.print("safetyCounter:    ");
     //Serial.print(safetyCounter);
     
     }

