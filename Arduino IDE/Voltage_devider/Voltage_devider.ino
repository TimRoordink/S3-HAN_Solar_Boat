//Watch the video instruction for this sketch:https://youtu.be/t8xwrVj2aFs

const float arduinoVCC = 3.00;//Your Arduino voltage
unsigned long ValueR1 = 2700;
unsigned long ValueR2 = 56000;
const int analogPinVoltageDiv = A0;//the pin connecting the voltage 
const int analogPinCurrentSens = A1;//the pin connecting the current
const int inputResolution =4095;//works with most Arduino boards
const float average_of = 500;//Average of 500 readings
const int max_current = 10;//The max current that can be measured

float voltage;
float current;
int A1Value;
int A0Value;

void setup() {
  Serial.begin(9600);
  Serial.println("Reading voltage with Arduino");
  delay(500);
}

void loop() {
  readVoltage();
//  VoltageAverage();
//  readCurrent()
  delay(100);
}

void readVoltage(){
    int A0Value = analogRead(A0);
    Serial.println(A0Value);
  
//    int A0Value = analogRead(analogPinVoltageDiv);
//    float voltage_sensed = A0Value * (arduinoVCC / (float)inputResolution); 
//    voltage = voltage_sensed * ( 1 + ( (float) ValueR2 /  (float) ValueR1) );
//    //Serial.print("voltage_sensed:");
//    //Serial.println(voltage);
}

void VoltageAverage(){
    float voltage_temp_average=0;
    for(int i=0; i < average_of; i++)
    {
       readVoltage();
       voltage_temp_average +=voltage;
    }
    Serial.print("voltage_average:");
    Serial.println(voltage);
}

void readCurrent(){
    int A1Value = analogRead(analogPinCurrentSens);
    float current = (A1Value / (float)inputResolution) * max_current;    
}
