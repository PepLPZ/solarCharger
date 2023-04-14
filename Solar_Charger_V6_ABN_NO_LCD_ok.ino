////////////////////////////////PWM SOLAR CHARGE CONTROLLER V-6.0 NO LCD///////////////////////////////
//----------------------------------------------------------------------------------------------------

  //  Author: JOSEP LOPEZ SANCHEZ
  //          www.ea3abn.com
  //  This code is for an arduino based Solar PWM charge controller ( V-2.0)
  //  Credit :logic of 3 stage charging taken from http://jamesoid.com/pwm_charger_project/pwm_charger.ph
  //  Last updated on 13/04/2023
  //----------------------------------------------------------------------------------------------------
  // Arduino pins Connections----------------------------------------------------------------------------
  
    // A0 - Voltage divider to measure solar panel voltage
    // A1 - Voltage divider to measure battery voltage
    // A2 - ACS712 to monitor load current
    // A3 - ACS712 to monitor solar current
    // A4 - LCD SDA 
    // A5 - LCD SCL
    // D0 - Not used
    // D1 - Not used
    // D2 - Control Load MOSFET Q2 
    // D3 - PWM Output to control MOSFET Q1 
    // D4 - Push Switch Control LOAD (ADDED)<<<<<<<<<<<<<<<
    // D5 - Battery Red LED       
    // D6 - Battery Green LED 
    // D7 - Battery Blue LED 
    // D8 - Load Red Led 
    // D9 - Load Green Led  
    // D10- Solar Red LED
    // D11- Solar Green LED  
    // D12- DS18B20 Temp. Sensor
    // D13-not used  

#include <Wire.h>
#include <OneWire.h>  
#include <DallasTemperature.h> 

#define SOL_ADC A0     // Solar panel side voltage divider is connected to pin A0 
#define BAT_ADC A1     // Battery side voltage divider is connected to pin A1
#define LOAD_CURRENT_ADC A2  // ACS 712 current sensor is connected to pin A2 for load curremt
#define SOL_CURRENT_ADC A3  // ACS 712 current sensor is connected to pin A3 for solar current
#define AVG_NUM 10    // number of iterations of the adc routine to average the adc readings
#define BAT_MIN 10.5  // minimum battery voltage for 12V system "CHANGED default BAT_MIN 10.5"
#define BAT_MAX 14.6  // maximum battery voltage for 12V system "CHANGED default BAT_MAX 15.0"
#define BULK_CH_SP 14.6 // bulk charge set point for sealed lead acid battery // flooded type set it to 14.6V
#define FLOAT_CH_SP 13.6  //float charge set point for lead acid battery
#define LVD 11.5       //Low voltage disconnect setting for a 12V system default (11.5)
#define PWM_PIN 3      // pin-3 is used to control the charging MOSFET //the default frequency is 490.20Hz
#define LOAD_PIN 2     // pin-2 is used to control the load
#define LOAD_SWITCH 4  // pin-4 is used to SWICH ON OR OFF THE LOAD (PEP)<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define BAT_RED_LED 5
#define BAT_GREEN_LED 6
#define BAT_BLUE_LED 7
#define LOAD_RED_LED 8
#define LOAD_GREEN_LED 9
#define SOL_RED_LED 10
#define SOL_GREEN_LED 11
#define ONE_WIRE_BUS 12 // Data wire of DS18B20 temp. sensor is connected to pin 12  

//--------------------------------------------------------------------------------------------------------------------------
///////////////////////DECLARACION DE TODAS LAS VARIABLES GLOBALES /////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------
float solar_volt=0;
float bat_volt=0;
float load_current=0;
float solar_current=0;
float offsetVoltage = 2.5; // for ACS712 sensor
float Sensitivity =0.100; // 100mV/A for ACS712-20A variant
int temperature=0;
int temp_change=0;
float system_volt=0;
float bulk_charge_sp=0;
float float_charge_sp=0;
int charge_stage =0;
int load_status=0;///////////////////////////////////////////////////////////////////////////////////////
double PWM_Duty =0; // PWM_Duty varies from 0 to 255
float lvd; // <<<<<<<<<<<<<<<<Almacenamos el valor del voltage minimo de desconexion para bateria de 12v
int Biestable=0; // VARIABLE PARA LA CONMUTACION del Switch ON/OFF

// Variables Para las Medidas de Potencia y Energia
long unsigned time =0; 
long unsigned msec=0;
long unsigned last_msec=0;
long unsigned elasped_msec=0;
long unsigned elasped_time=0;
float load_ampSecs = 0;
float load_ampHours=0;
float load_watts=0;
float load_wattSecs = 0;
float load_wattHours=0;
float solar_ampSecs = 0;
float solar_ampHours=0;
float solar_watts=0;
float solar_wattSecs = 0;
float solar_wattHours=0;

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor 
//******************************************************* MAIN PROGRAM START ************************
void setup()
{

Serial.begin(9600);
sensors.begin(); // // Start up the library  
TCCR2B = TCCR2B & B11111000 | 0x03;    // pin 3 PWM frequency of 980.39 Hz

pinMode(SOL_RED_LED ,OUTPUT);
pinMode(SOL_GREEN_LED,OUTPUT);
pinMode(BAT_RED_LED,OUTPUT);
pinMode(BAT_GREEN_LED,OUTPUT);
pinMode(BAT_BLUE_LED,OUTPUT);
pinMode(LOAD_RED_LED ,OUTPUT);
pinMode(LOAD_GREEN_LED,OUTPUT);
pinMode(PWM_PIN,OUTPUT);
pinMode(LOAD_PIN,OUTPUT);   //<<<<< SALIDA DE CARGA
pinMode(LOAD_SWITCH,INPUT); // Define pinD4 input to control the load ON/OFF <<<<< PULSADOR CARGA
digitalWrite(PWM_PIN,LOW);  // default value of pwm duty cycle
digitalWrite(LOAD_PIN,HIGH);  // default load state is ON CADA VEZ QUE ARRANCAMOS
//digitalWrite(LOAD_GREEN_LED,HIGH);// default load led state is ON CADA VEZ QUE ARRANCAMOS
}

void loop()
{
 load_Push();             // Lectura del pulsador
 read_data();             // read different sensors data from analog pin of arduino
 system_voltage();        // detect the system voltage according to battery voltage
 setpoint();              // decide the charge set point according to system voltage
 charge_cycle();          // pwm charging of battery
 power();                 // calculate the load power and energy
 led_indication();        // led indication
 print_data();            // print in serial monitor
 }
//************************************************************ PROGRAM END ****************************


//------------------------------------------------------------------------------------------------------
////////////////// READS AND AVERAGES THE ANALOG INPUTS (SOLAR VOLTAGE,BATTERY VOLTAGE)////////////////
//------------------------------------------------------------------------------------------------------
int read_adc(int adc_parameter)
{
  
  int sum = 0;
  int sample ;
  for (int i=0; i<AVG_NUM; i++) 
  {                                        // loop through reading raw adc values AVG_NUM number of times  
    sample = analogRead(adc_parameter);    // read the input pin  
    sum += sample;                        // store sum for averaging
    delayMicroseconds(50);              // pauses for 50 microseconds  
  }
  return(sum / AVG_NUM);                // divide sum by AVG_NUM to get average and return it
}
//-------------------------------------------------------------------------------------------
///////////////////////////////////LOAD CONTROL SWITCH ON/OFF BY PP/////////////////////////
//------------------------------------------------------------------------------------------- 
void load_Push(void)
{  
  // Encender la carga si pulsamos el pulsador y la variable Biestable es igUal a cero  
  {
 if (digitalRead(LOAD_SWITCH)==HIGH && Biestable==0 && load_status==0)
  {
   Biestable=1;
   load_status=1;  
   digitalWrite(LOAD_PIN, HIGH); // load is ON
   delay(10);
  }
  // Apagar LED si pulsamos el pulsador y la variable biestable es igUal a uno
   else if (digitalRead(LOAD_SWITCH)==HIGH && Biestable==1 && load_status==1)
  {
   Biestable=0;// Cambiamos valor de variable a 0
   load_status=0; 
   digitalWrite(LOAD_PIN, LOW); //load is OFF
   delay(10);
  }
  }
} 
//--------------------------------------------------------------------------------------------
////////////////////////////////////READ THE DATA/////////////////////////////////////////////
//--------------------------------------------------------------------------------------------
 void read_data(void) 
 {
   // 5V = ADC value 1024 => 1 ADC value = (5/1024)Volt= 0.0048828Volt
   // Vout=Vin*R2/(R1+R2) => Vin = Vout*(R1+R2)/R2   R1=100 and R2=20
    
     solar_volt = read_adc(SOL_ADC)*0.00500*(120/20);/// valor por defecto 0.00488 valor corregido *0.00540*
     bat_volt   = read_adc(BAT_ADC)*0.00490*(120/20);/// valor por defecto 0.00488 valor corregido *0.00580*
     load_current = ((read_adc(LOAD_CURRENT_ADC)*0.00488 - offsetVoltage -0.05)/Sensitivity ); // 2.4V offset when no load is connected
     solar_current = ((read_adc(SOL_CURRENT_ADC)*0.00488 - offsetVoltage)/ Sensitivity ); 
      if (load_current <0)
     {
      load_current = 0;
     }
     if (solar_current <0)
     {
      solar_current = 0;
     }
    
    sensors.requestTemperatures();  // get temperature readings 
    temperature = sensors.getTempCByIndex(0) ;    // 0 refers to the first IC on the wire 
     
  }
//----------------------------------------------------------------------------------------
/////////////////////////////////POWER AND ENERGY CALCULATION //////////////////////////////
//------------------------------------------------------------------------------------------
void power(void){
  
msec = millis();
elasped_msec = msec - last_msec; //Calculate how long has past since last call of this function
elasped_time = elasped_msec / 1000.0; // 1sec=1000 msec
load_watts = load_current * bat_volt; //Watts now
solar_watts = solar_current * solar_volt; //Watts now

load_ampSecs = (load_current*elasped_time); //AmpSecs since last measurement
solar_ampSecs = (solar_current*elasped_time); //AmpSecs since last measurement

load_wattSecs = load_ampSecs * bat_volt; //WattSecs since last measurement
solar_wattSecs = solar_ampSecs * solar_volt; //WattSecs since last measurement

load_ampHours = load_ampHours + load_ampSecs/3600; // 1 hour=3600sec //Total ampHours since program started
solar_ampHours = solar_ampHours + solar_ampSecs/3600; // 1 hour=3600sec //Total ampHours since program started

load_wattHours = load_wattHours + load_wattSecs/3600; // 1 hour=3600sec //Total wattHours since program started
solar_wattHours = solar_wattHours + solar_wattSecs/3600; // 1 hour=3600sec //Total wattHours since program started

last_msec = msec; //Store 'now' for next time
}
//------------------------------------------------------------------------------------------------------------
/////////////////////////////////ENVIO DE TRAMA AL MODULO LORA //////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
  void print_data(void) 
  {
    delay(100);//            ojo
    Serial.print("  POSTE-2 ZONA1  ");
//    Serial.println; 
    Serial.print(" V-PANEL: ");  
    Serial.print(solar_volt);
//    Serial.println;
    Serial.print(" BAT: "); 
    Serial.print(bat_volt);
//    Serial.println;
    Serial.print(" Temp Bat: " );
    Serial.print (temperature);
//    Serial.println;
    Serial.print("  I-PANEL: ");
    Serial.print(solar_current);
//    Serial.println;
    Serial.print("A");
    Serial.print("  I-SALIDA: ");
//    Serial.println;
    Serial.print(load_current);
    Serial.println("A");
 //    Serial.println;
 //   Serial.print("LA CARGA ESTA:");
 //   Serial.print(Biestable);
 }
//----------------------------------------------------------------------------------------------------------------------
//////////////////////////////////SYSTEM VOLTAGE AUTO DETECT ///////////////////////////////////////////////////////////
//----------------------------------------------------------------------------------------------------------------------
void system_voltage(void)
{
  if ((bat_volt >BAT_MIN) && (bat_volt < BAT_MAX))
  {
     system_volt = 12;
  } 
}
//-----------------------------------------------------------------------------------------------------------------------
 ////////////////////////////////////CHARGE SET POINT ///////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------------------------------------------------
 
void setpoint(void)
{
  temp_change =temperature-25.0; // 25deg cel is taken as standard room temperature 
  // temperature compensation = -5mv/degC/Cell 
  // If temperature is above the room temp ;Charge set point should reduced
  // If temperature is bellow the room temp ;Charge set point should increased
  if(system_volt ==12)
  {
     bulk_charge_sp = BULK_CH_SP-(0.050*temp_change) ;
     float_charge_sp=FLOAT_CH_SP-(0.050*temp_change) ;
     lvd =LVD;
  }
 
  else if(system_volt ==6)
  {
     bulk_charge_sp = (BULK_CH_SP/2)-(0.015*temp_change) ;
     float_charge_sp= (FLOAT_CH_SP/2)-(0.015*temp_change) ;
     lvd=LVD/2;
  }
}
//---------------------------------------------------------------------------------------------
 ///////////////////////////////////////////////////PWM CHARGE CYCLE @500 HZ //////////////////
 //--------------------------------------------------------------------------------------------
void charge_cycle(void)
{
  if(solar_volt <= 10){ 
    read_data();         // Reading voltage temp from the sensors
    setpoint();              // Reading temp compensated charging set point
    PWM_Duty = 0;
    analogWrite(PWM_PIN,PWM_Duty);  //generate PWM from D9 @ 0% duty // Shut down the charger
    
    }
 

//----------------------------------------------------------------------------------------------
///////////////////////////////////////////////////PWM CHARGE CYCLE @500 HZ ////////////////////
//----------------------------------------------------------------------------------------------

///////////////////////////////////  STAGE-1 (BULK CHARGING)////////////////////////////////////
    // During this stage the MOSFET is fully on by setting the duty cycle to 100%
    // Constant Current Charging
    ////////////////////////////////////////////////////////////////////////////////////////////  
  if((bat_volt < bulk_charge_sp) && (solar_volt> (bat_volt + .5)) && charge_stage == 1){
    read_data();        // Reading voltage and temp from the sensors
    setpoint();               // Reading temp compensated charging set point
    PWM_Duty = 252.45;
    analogWrite(PWM_PIN,PWM_Duty); // 99 % duty cycle // rapid charging   
   }
    charge_stage = 2;
    

///////////////////////////////////  STAGE-2 (TOPPING CHARGE)///////////////////////////
// During this stage the MOSFET is fully on by setting the duty cycle to 100%
// Constant voltage
////////////////////////////////////////////////////////////////////////////////////////
  
   // 25.5 is 10 % duty cycle
    if((PWM_Duty> 25.5 ) && (solar_volt > (bulk_charge_sp + .5)) && charge_stage == 2){
    time = millis();  
     read_data();         // Reading voltage and temp from the sensors
     setpoint();              // Reading temp compensated charging set point
    
    if(bat_volt >= bulk_charge_sp){ // if the battery voltage rise above the bulk charge setpoint 
    PWM_Duty--;                     // Slow down the charging rate by reducing the pwm duty cycle
    if(PWM_Duty < 0){               // if during slow down process duty cycle falls below 0% then set to 0%
    PWM_Duty=0;
    }
     analogWrite(PWM_PIN,PWM_Duty);    //generate PWM from D3 @ 0% duty // MOSFET Q1 is OFF
    }
    if(bat_volt < bulk_charge_sp){ // if the battery voltage falls below the bulk charge setpoint, then go to stage-1
    charge_stage = 1;
    }  
    else{
      analogWrite(PWM_PIN,PWM_Duty);  
    }
 
  /* while( millis()-time > 3600000){ // Ensure the charger stay 1hour in bulk charging 
    charge_stage = 3;
   }
   */
    
    }
   charge_stage = 3; 
 ///////////////////////////////////  STAGE-3 (FLOAT CHARGING)///////////////////////////////////////
 // During this stage the MOSFET is fully on by setting the duty cycle to 100%
 // Constant Current Charging
 ////////////////////////////////////////////////////////////////////////////////////////////////////
  
    if((solar_volt > ( float_charge_sp + .5)) && charge_stage == 3){
    read_data();         // Reading voltage and temp from the sensors
    setpoint();      // Reading temp compensated charging set point
    if(bat_volt >= float_charge_sp){ // if the battery voltage is more than float charge setpoint then shut down the charger
    PWM_Duty = 0;                    // setting duty cycle =0 to turn off the MOSFET Q1
    analogWrite(PWM_PIN,PWM_Duty);  //generate PWM from D9 @ 0% duty // MOSFET Q1 is OFF
    }
    else if(bat_volt <  float_charge_sp - 0.5){ // if the battery voltage falls below the float charge setpoint then go to stage -1
    charge_stage = 1;
    }
    else{
    PWM_Duty = 252.25; // setting duty cycle = 5% for trickle charge
    
    analogWrite(PWM_PIN,PWM_Duty);  //generate PWM from D9 @ 5% duty // Q1 is driving @ 5% duty cycle
   
    } 
    
   }
  }

//--------------------------------------------------------------------------
//////////////////////////LED INDICATION////////////////////////////////////
//--------------------------------------------------------------------------
void led_indication(void)
{
  solar_led();             //Solar led indication
  battery_led();           //Battery status led indication
  load_led();              //Load led indication
}
//-----------------------------------------------------------------------------------
/////////////////////////////////////////////SOLAR LED INDICATION///////////////////
//----------------------------------------------------------------------------------  
  
  void solar_led()
  
  {
    if (solar_volt > float_charge_sp)
  {  
      digitalWrite(SOL_RED_LED,LOW);
      digitalWrite(SOL_GREEN_LED,HIGH);  // Sun Light is available and charger is ready for charging
  } 
  else 
  {
      digitalWrite(SOL_GREEN_LED,LOW);
      digitalWrite(SOL_RED_LED,HIGH);  //Sun light is not available for charging
      
  }  
     }
//-------------------------------------------------------------------------------------
/////////////////////////////////////////////BATTERY LED INDICATION////////////////////
//-------------------------------------------------------------------------------------
void battery_led(void)
{
  
   if( (bat_volt > system_volt) && ( bat_volt <bulk_charge_sp))
  {   
      leds_off_all();
      digitalWrite(BAT_GREEN_LED,HIGH);  // battery voltage is healthy
  } 
  else if(bat_volt >= bulk_charge_sp) 
  {
      leds_off_all();
      digitalWrite(BAT_BLUE_LED,HIGH);  //battery is fully charged
  }
   else if(bat_volt < system_volt)
  {
      leds_off_all();
      digitalWrite(BAT_RED_LED,HIGH);  // battery voltage low
  }
}
//----------------------------------------------------------
//////////////////////////LOAD LED INDICATION///////////////
//---------------------------------------------------------- 
   void load_led()

 {
    if(load_status==1)
    {
      digitalWrite(LOAD_RED_LED, LOW);
      digitalWrite(LOAD_GREEN_LED,HIGH);
    }
    else if(load_status==0)
    {
      digitalWrite(LOAD_GREEN_LED, LOW);
      digitalWrite(LOAD_RED_LED,HIGH);
    }
}
//----------------------------------------------------------------------
//////////////////////// TURN OFF ALL THE LED///////////////////////////
//----------------------------------------------------------------------
void leds_off_all(void)

{ 
  
  digitalWrite(BAT_RED_LED,LOW);
  digitalWrite(BAT_GREEN_LED,LOW);
  digitalWrite(BAT_BLUE_LED,LOW);
  
}
