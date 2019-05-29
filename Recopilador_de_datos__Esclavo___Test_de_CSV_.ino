// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <math.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.

   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // PIN para interrupción del MPU6050
#define PIN_RPM 3
#define RPM_SEGUNDOS 1.0 // Cantidad de tiempo máximo para medir RPM
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3] = {0.0, 0.0, 0.0};         // [psi, theta, phi]    Euler angle container
float ypr[3] = {0.000, 0.000, 0.000};           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float VelocidadAngular[3] = {0.0, 0.0, 0.0};           // [y, x, z]
float AceleracionAngular[3] = {0.0, 0.0, 0.0};         // [y, x, z]
float yprVariacion[3] =  {0.0, 0.0, 0.0};         // [y, x, z]

float brazo = 115;
float height = 85.91;
float altura, alturaant=0, opuesto, varposicion;
float angulo;
int estable, chequeo, primeropuesto, firsttr;


float ypr_ant[3] = {0.0, 0.0, 0.0};           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float VelocidadAngular_ant[3] = {0.0, 0.0, 0.0};           // [y, x, z]
float AceleracionAngular_ant[3] = {0.0, 0.0, 0.0};         // [y, x, z]

// Intensidad de Corriente
float ICSensibilidad=0.228; //sensibilidad en V/A para nuestro sensor
float ICOffset=0.043; //sensibilidad en V/A para nuestro sensor
float I = 0.0;

long tiempo_inicial = 0;
double tiempo_inicial_seg;
long tiempo_final = 0;
double tiempo_final_seg;
double tiempo_total_seg;
unsigned int inicializar = 1;

int comienzo_delay = 3000;

// ================================================================
// ===                     RUTINAS AUXILIARES                   ===
// ================================================================
// Intensidad de Corriente
float get_corriente(int n_muestras)
{
  float voltajeSensor;
  float corriente=0;
  for(int i=0;i<n_muestras;i++)
  {
    voltajeSensor = analogRead(A0) * (5.0 / 1023.0);////lectura del sensor
    corriente=corriente+(voltajeSensor-2.5)/ICSensibilidad; //Ecuación  para obtener la corriente
  }
  corriente = (corriente/n_muestras) - ICOffset;
  if( corriente < 0.0 ) {
    corriente = 0;
  }
  
  return(corriente);
}


// Revoluciones por Minuto
double rpm;
double rpmPromedio;
volatile int rpm_sumVueltas = 0;
int rpm_sumVueltasAcumulado = 0;
const unsigned long rpm_sampleTime = 500;

void sumRPM() {
  rpm_sumVueltas = rpm_sumVueltas + 1;
}

float getRPM()
{
  int rpm_count=0;
  boolean rpm_flag=LOW;
  unsigned long rpm_currentTime=0;
  unsigned long rpm_startTime=millis();
  
  while (rpm_currentTime<=rpm_sampleTime)
  {
    if (digitalRead(PIN_RPM)==HIGH)
    {  
      rpm_flag=HIGH;
    }
    if (digitalRead(PIN_RPM)==LOW && rpm_flag==HIGH)
    {
      rpm_count++;
      rpm_flag=LOW;
    }
    rpm_currentTime=millis()-rpm_startTime;
  }
  // int kount2rpm = int(60000./float(sampleTime))*kount;

  float rmp_total = (60.00*rpm_count)/(rpm_sampleTime/1000);
  return rmp_total;
}



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
    delay(comienzo_delay);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
//        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    // Serial.println(F("Testing device connections..."));
    // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    // Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(240);
    mpu.setYGyroOffset(29);
    mpu.setZGyroOffset(-44);
    mpu.setZAccelOffset(777); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        // attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        pinMode(PIN_RPM, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_RPM), sumRPM, FALLING);
/*
        Serial.println("***************************");
        Serial.println(" Grupo Undimotriz - UTN.BA");
        Serial.println("***************************");
        Serial.println("Por favor, esperar 3 segundos para comenzar con la captura de datos");
        Serial.println("*******************************************************************");
*/
     //   Serial.println("tiempo_inicial \t tiempo_final \t delta_tiempo \t corriente \t RPM \t RPM_Prom \t altura \t varposicion \t anguloY \t anguloX \t varY \t varX \t velY \t velX \t accY \t accX");
     //   Serial.println("seg \t seg \t seg \t A \t rpm \t rpm \t cm \t cm \t rad \t rad \t rad \t rad \t rad/seg \t rad/seg \t rad/seg2 \t rad/seg2");
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
  /////////////////////////////////////////////////////////////////////
  // ===  Delay to adjust the speed of the PC with PROCESSING 2   === [J_RPM]
  // ...if the processor in your PC is fast, you can eliminate this delay
  /////////////////////////////////////////////////////////////////////
  mpu.resetFIFO();

  // Copiar valores actuales a las variables del ciclo anterior
  ypr_ant[0] = ypr[0];
  ypr_ant[1] = ypr[1];
  ypr_ant[2] = ypr[2];
  VelocidadAngular_ant[0] = VelocidadAngular[0];
  VelocidadAngular_ant[1] = VelocidadAngular[1];
  VelocidadAngular_ant[2] = VelocidadAngular[2];
  AceleracionAngular_ant[0] = AceleracionAngular[0];
  AceleracionAngular_ant[1] = AceleracionAngular[1];
  AceleracionAngular_ant[2] = AceleracionAngular[2];


  // Tiempo en el que finalizó el ciclo anterior
  tiempo_inicial = tiempo_final;
  tiempo_inicial_seg = (double)(tiempo_inicial)/1000.0;


    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    /*
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
    */

    
    // reset interrupt flag and get INT_STATUS byte
    // mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO co  unt
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Obtención de información de MPU6050
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        // Cálculo de posición con respecto al brazo en reposo

        angulo = - ( ypr[1] * 180/M_PI); // Calculo en ángulo en grados en el que está el acelerómetro
        altura = -((- sin (ypr[1]) * brazo)- height); // Calculo la altura en la que está la punta del brazo, en relación al 0 (que es con el brazo en reposo)
        opuesto = (- sin (ypr[1]) * brazo); // Calculo auxiliar para sacar la medida del lado opuesto, para el triangulo formado por dicho angulo.


        // Cálculo de variación de posición respecto a un estado anterior
        
        if (primeropuesto == 0) { //Bucle auxiliar para antes de calcular la primera variación, poner ambas variables con el mismo valor
                      alturaant = opuesto;
                      primeropuesto = 1;
              }
        
        varposicion = -( opuesto - alturaant); // Resto el lado opuesto inicial, con el nuevo lado opuesto (hubo una variación de altura en el eje vertical)
      //  varposicion = round (varposicion); // Redondeamos el valor
        alturaant = opuesto; // Guardamos el valor para el próximo bucle

        
        yprVariacion[0] = ypr[0] - ypr_ant[0];
        yprVariacion[1] = ypr[1] - ypr_ant[1];

        // Delay para dar tiempo en la obtenció|n de RPM
        delay(200);

        // Tiempo Final
        tiempo_final = millis();
        tiempo_final_seg = (double)(tiempo_final)/1000.0;
      
        tiempo_total_seg = (double) (tiempo_final_seg - tiempo_inicial_seg);
        
        // Velocidad Angular
        VelocidadAngular[0] = yprVariacion[0]/tiempo_total_seg;
        VelocidadAngular[1] = yprVariacion[1]/tiempo_total_seg;

        // Aceleración Angular
        AceleracionAngular[0] = (VelocidadAngular[0]-VelocidadAngular_ant[0])/tiempo_total_seg;
        AceleracionAngular[1] = (VelocidadAngular[1]-VelocidadAngular_ant[1])/tiempo_total_seg;

        // Intensidad de Corriente
        I = get_corriente(200); // Obtener el promedio de 200 lecturas.
      
        // rpm_tiempo_total_seg = 0.0;
        // RPM
        detachInterrupt(digitalPinToInterrupt(PIN_RPM));
        rpm = (double)((60.0 * rpm_sumVueltas)/tiempo_total_seg);
        rpm_sumVueltasAcumulado = rpm_sumVueltasAcumulado + rpm_sumVueltas;
        rpmPromedio = (double)((60.0 * rpm_sumVueltasAcumulado)/tiempo_final_seg);
        rpm_sumVueltas = 0;
        attachInterrupt(digitalPinToInterrupt(PIN_RPM), sumRPM, FALLING);

        //Empieza transmisión desde 0

        if (firsttr == 0){

        Serial.print("&");
        firsttr = 1;

        }

        // Imprimir datos obtenidos
        Serial.print(tiempo_inicial_seg);
        Serial.print(",");
        Serial.print(tiempo_final_seg);
        Serial.print(",");
        Serial.print(tiempo_total_seg);
        Serial.print(",");

        Serial.print(I);
        Serial.print(",");

        Serial.print(rpm);
        // Serial.print(" rpm\t");
        Serial.print(",");

        Serial.print(rpmPromedio); 
        // Serial.print(" rpmPrm\t"); 
        Serial.print(",");

        Serial.print(altura);
        // Serial.print(" altura\t");
        Serial.print(",");

        Serial.print(varposicion);
        // Serial.print("varaltura\t");
        Serial.print(",");
        
        Serial.print(ypr[0] * 180/M_PI);
        // Serial.print(" Y\t");
        Serial.print(",");
        Serial.print(- ypr[1] * 180/M_PI);
        // Serial.print(" X\t");
        Serial.print(",");

        Serial.print(abs(yprVariacion[0]));
        // Serial.print(" varY\t");
        Serial.print(",");
        Serial.print(abs(yprVariacion[1]));
        // Serial.print(" varX\t");
        Serial.print(",");

        Serial.print(abs(VelocidadAngular[0]));
        // Serial.print(" vY\t");
        Serial.print(",");
        Serial.print(abs(VelocidadAngular[1]));
        // Serial.print(" vX\t");
        Serial.print(",");

        Serial.print(abs(AceleracionAngular[0]));
        // Serial.print(" aY\t");
        Serial.print(",");
        Serial.println(abs(AceleracionAngular[1]));
        // Serial.print(" aX");
       // Serial.print(",");
        
       // Serial.println("");
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
