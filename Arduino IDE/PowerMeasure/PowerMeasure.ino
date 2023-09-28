const int voltagePin = A0;            //Pin connecting voltage devider
const int ampsPin = A1;               //Pin connecting current measure module
const float arduinoVCC = 3.00;        //ADC internal voltage reference
const int inputResolution = 4095;     //12 bit ADC resolution for L432KC
const unsigned long valueR1 = 2700;   //Value first voltage devider resistor
const unsigned long valueR2 = 56000;  //Value second voltage devider resistor
int A0Value;                          //Raw A0 value
int A1Value;                          //Raw A1 value
float voltageSensed;                  //Voltage measured on A0
float voltage;                        //Measured voltage

float sensitivity = 1000.0 / 264.0;   //Sensitivty for the current sensor
float Vref = -312.50;                 //Reference voltage (needs change)
float unitValue;                      //Conversion factor
float ampsVoltage;                    //Reference voltage
float current;                        //Measured current

float power;                          //Calculated power

void setup() {
  Serial.begin(9600);                                             //Start serial connection
  Serial.println("Reading voltage, amps and calculating power");  //Print line in serial monitor
  delay(500);                                                     //Wait
}

void loop() {
  readVoltage();
  readAmps();
  calculatePower();
  printValues();
  delay(100); //Wait
}

void readVoltage() {
  A0Value = analogRead(voltagePin);                           //Get raw A0 value
  voltageSensed = A0Value * ( arduinoVCC / inputResolution ); //Convert A0 value back to voltage on pin
  voltage = voltageSensed * ( 1 + ( valueR2 / valueR1 ) );    //Calculate the voltage using the voltage devider ratio and the voltage on pin
}

void readAmps() {
  A1Value = analogRead(ampsPin);                             //Get raw A1 value
  unitValue = arduinoVCC / inputResolution * 1000.0;         //Calculate the conversion factor from the digital value to milivolts
  ampsVoltage = unitValue * A1Value;                         //Calculate the voltage across the current sensor
  current = ( ampsVoltage - Vref ) * sensitivity;            //Calculate the current using the correct offset
}

void calculatePower() {
  power = voltage * current;
}

void printValues() {
  Serial.print("A0value=");
  Serial.print(A0Value);

  Serial.print("\t");

  Serial.print("voltageSensed=");
  Serial.print(voltageSensed);

  Serial.print("\t");

  Serial.print("voltage=");
  Serial.print(voltage);

  Serial.print("\t");

  Serial.print("A1value=");
  Serial.print(A1Value);

  Serial.print("\t");

  Serial.print("ampsVoltage=");
  Serial.print(ampsVoltage);

  Serial.print("\t");

  Serial.print("current=");
  Serial.println(current);
}
