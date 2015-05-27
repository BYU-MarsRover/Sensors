/*
 # This sample code is used to test the pH meter V1.0.
 # Editor : YouYou
 # Ver    : 1.0
 # Product: analog pH meter
 # SKU    : SEN0161
*/
// Video mux servo control
#include <Servo.h>

#define pH_Pin A0            //pH meter Analog output to Arduino Analog Input 0
#define sm_Pin A1
#define pH_Offset -0.45            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 900
#define ArrayLength  40    //times of collection

Servo mux_pwm;
int mux_laser;
int mux_pos;
int laser = 10;
int laser_int;


double sm_Offset = 0.00;
int pHArray[ArrayLength];   //Store the average value of the sensor feedback
int smArray[ArrayLength];
int pHArrayIndex=0;
int smArrayIndex=0;
double temp_input = 0.00;
double sm_in_air = 1022.00;
double sm_in_water = 150.00;

//Class flasher
class Flasher
{
    int ledPin;
    long OnTime;
    long OffTime;
  
    int ledState;
    unsigned long prev_millis;
    
    int enabled;
  
  public:
  Flasher(int pin, long on, long off)
  {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);
    
    OnTime = on;
    OffTime = off;
    
    ledState = LOW;
    prev_millis = 0;
    
    enabled = 0;
  }
  
  void enable(int on_off)
  {
    enabled = on_off;
  }
  
  void update()
  {
    //something to say if enabled change if not enabled dont change
    if(enabled == 0){
      digitalWrite(ledPin, LOW);
      return;
    }
    unsigned long currentMillis = millis();
    
    if((ledState == HIGH) && (currentMillis - prev_millis >= OnTime))
    {
      ledState = LOW;  // Turn it off
      prev_millis = currentMillis;  // Remember the time
      digitalWrite(ledPin, ledState);  // Update the actual LED
    }
    else if ((ledState == LOW) && (currentMillis - prev_millis >= OffTime))
    {
      ledState = HIGH;  // turn it on
      prev_millis = currentMillis;   // Remember the time
      digitalWrite(ledPin, ledState);	  // Update the actual LED
    }
  }
};


Flasher laser1(10,500,500);
void setup(void)
{
  pinMode(laser,OUTPUT);  
  temp_input = 950/((1/sm_in_water)-(1/sm_in_air));
  sm_Offset = -temp_input/sm_in_air;
  //Serial.begin(9600);  
  Serial.begin(115200);
    //Serial1.begin(115200);
  // This is commented out because we only want to send back the actually values
  //Serial.println("pH and soil moisture meter experiment!");    //Test the serial monitor
  
  // Mux pwm setup
  mux_pwm.attach(9);
  mux_pwm.write(1);
}

int last_video =0;

void loop(void)
{
  //flasher update
  laser1.update();
  
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,pH_voltage;
  static float smValue,sm_voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(pH_Pin);
      smArray[smArrayIndex++]=analogRead(sm_Pin);
      if(pHArrayIndex==ArrayLength)pHArrayIndex=0;
      if(smArrayIndex==ArrayLength)smArrayIndex=0;
      pH_voltage = avergearray(pHArray, ArrayLength)*5.0/1024;
      sm_voltage = avergearray(smArray, ArrayLength);
      pHValue = 3.5*pH_voltage+pH_Offset;
      smValue = (temp_input/sm_voltage)+sm_Offset;
      //smValue = sm_voltage;  //For use to get initial water and air readings
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
        // All commented print lines are commented so we only 
	//Serial.print("pH Voltage:");
        //Serial.print(pH_voltage,2);
        //Serial.print("    pH value: ");
	//Serial.println(pHValue,2);
        //Serial.print(pHValue,2);
        //digitalWrite(LED,digitalRead(LED)^1);
        
        //Serial.print("soil Moisture Voltage:");
        //Serial.print(sm_voltage,2);
        //Serial.print("    Moisture Sensor Value:");
        //Serial.println(smValue,2);  
        //Serial.println(smValue,2);
        
        print_ph_hum_sensor_data(pHValue, smValue);
        printTime=millis();
        
  }
  if(Serial.available()){
      //mux_pos = Serial.parseInt();
      //Serial.println(mux_pos);
      //Serial.println(Serial.read());
      //digitalWrite(laser,digitalRead(laser)^1);
      mux_laser = Serial.read();
      //Serial.println(mux_laser, DEC);
      //Serial.println(mux_laser);
      //Serial.println(mux_laser);
      //laser_int = Serial.read();
      //Serial.println(mux_int);
      //Serial.println(laser_int);
      //int i = 0;
      //for(i = 0; i <180;i++){
       // mux_pwm.write(i);
       // Serial.println(i);
       // delay(50);
      //}
      // 0 2 6 - switch video 1, 2, 3 laser off
      // 1 3 7 - switch video 1, 2, 3 laser on
      switch(mux_laser){
        case 16:
        //case 49:
          last_video = 45;
          //digitalWrite(laser, LOW);
          laser1.enable(0);
          mux_pwm.write(45);
          break;
        case 32:
        //case 50:
          //digitalWrite(laser, LOW);
          last_video = 90;
          //laser1.enable(0);
          mux_pwm.write(90);
          break;
        case 48:
        //case 51:
          //digitalWrite(laser, LOW);
          last_video = 135;
          //laser1.enable(0);
          mux_pwm.write(135);
          break;
        case 17:
        //case 57:
          // add flasher code here
          //digitalWrite(laser, HIGH);
          //Serial.println(" ");
          last_video = 45;
          laser1.enable(1);
          mux_pwm.write(45);
          break;
        case 33:
          //digitalWrite(laser, HIGH);
          laser1.enable(1);
          last_video = 90;
          mux_pwm.write(90);
          break;
        case 49:
          //digitalWrite(laser, HIGH);
          laser1.enable(1);
          last_video = 135;
          mux_pwm.write(135);
          break;
        default:
          //digitalWrite(laser, LOW);
          //laser1.enable(0);
          mux_pwm.write(last_video);
          break;
      }
  }
}

void print_ph_hum_sensor_data(float pHValue, float smValue){
    // deal with pHValue set it to a fixed number of bytes
    if(pHValue < 10)
    {
      Serial.print("0");
      Serial.print(pHValue,2);
    }
    else
    {
      Serial.print(pHValue,2);  
    }
    //Serial.print(" ");
    
    // deal with smValue 
    if(smValue < 0)
    {
      Serial.print("0000.00");
      //Serial.print(smValue,2);
    }
    else if(smValue < 10)
    {
      Serial.print("000");
      Serial.print(smValue,2);  
    }
    else if(smValue < 100)
    {
      Serial.print("00");
      Serial.print(smValue,2); 
    }
    else if(smValue < 1000)
    {
      Serial.print("0");
      Serial.print(smValue,2); 
    }
    else
    {
      Serial.print(smValue,2);  
    }
    //Serial.println(" ");
    
}

double avergearray(int* arr, int number)
{
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0)
  {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5)
  {   //less than 5, calculated directly statistics
    for(i=0;i<number;i++)
    {
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }
  else
  {
    if(arr[0]<arr[1])
    {
      min = arr[0];max=arr[1];
    }
    else
    {
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++)
    {
      if(arr[i]<min)
      {
        amount+=min;        //arr<min
        min=arr[i];
      }
      else 
      {
        if(arr[i]>max)
        {
          amount+=max;    //arr>max
          max=arr[i];
        }
        else
        {
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}
