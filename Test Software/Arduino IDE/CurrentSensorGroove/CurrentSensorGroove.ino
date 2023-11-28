#define currentPin A0
#define RefVal 5.00
const int averageOf = 500;
int sensorValue = 0;
float sensitivity = 1000.0 / 264.0;
float Vref = -312.50;

void setup() {
  Serial.begin(9600);
  Serial.println("Current sensor");
}

void loop() {
  for (int i = 0; i < averageOf; i++) {
    sensorValue += analogRead(currentPin);
    delay(2);
  }
  sensorValue = sensorValue / averageOf;
  
  float unitValue = RefVal / 1024.0 * 1000;
  float voltage = unitValue * sensorValue;

  //when no load, vref = initailvalue
  Serial.print("initialValue: ");
  Serial.print(voltage);
  Serial.println("mV");

  float current = (voltage - Vref) * sensitivity;

  Serial.print(current);
  Serial.println("mA");
  Serial.print("\n");

  sensorValue = 0;
  delay(1000);//wait
}
