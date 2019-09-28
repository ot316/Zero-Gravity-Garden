#include <dht.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h> 

 
const int hallPin = 9;
int motorrelay = 13;
int hallState = 0;
const int enB = 10;
const int in3 = 8; // fan motor
const int in4 = 7; // fan motor
int potPin = A8;    // select the input pin for the potentiometer
int potval = 0;       // variable to store the value coming from the potentiometer
int relay = 5;   //relay
int buzzer = 1; //buzzer
int PHdriftLED = 35;
int WatertankLED = 37;
int Water;
int encoder1switch = 36;
int encoder2switch = 34;
bool encoderswitchflag;
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x3F for a 20 chars and 4 line display
dht DHT; //humidity sensor Library
#define DHT11_PIN 6 //humidity and temperature sensor
int Temperature; //set temperature type
int Humidity; //set humidity type
#define encoder1PinA  18 //water cycle frequency rotary encoder
#define encoder1PinB  19 //water cycle frequency rotary encoder
#define encoder2PinA  2  //day cycle frequency rotary encoder
#define encoder2PinB  3  //day cycle frequency rotary encoder
#define SensorPin A3          //pH meter Analog output to Arduino Analog Input 3
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10],temp;
volatile float encoder1Pos = 0; 
volatile float encoder2Pos = 0;
float waterfreqhrs = 0.1;
int daycyclepercent = 100;
float daycyclescaler;
float nightcyclescaler;
unsigned long waterfreqmillis; //water cycle frequency in milliseconds
unsigned long previousMillis1=0; //set timer for water cycle
unsigned long previousMillis2=0; //set timer for day cycle
unsigned long previousMillis3=0; //set timer for night cycle
bool daynight = false;

void setup(){
  Serial.begin(9600);// initialize the serial port:
  pinMode(hallPin, INPUT);//hall effect sensor input
  pinMode(enB, OUTPUT);//fan motor pwm
  pinMode(in3, OUTPUT);//fan motor
  pinMode(in4, OUTPUT);//fan motor
  pinMode(relay, OUTPUT);//relay
  pinMode(motorrelay, OUTPUT);//MOTORRELAY
  pinMode(buzzer, OUTPUT);//Buzzer
  lcd.init();                      // initialize the lcd 
  lcd.backlight();                 //turn on backlight
  pinMode(encoder1PinA, INPUT);         // water cycle frequency rotary encoder
  digitalWrite(encoder1PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);       // turn on pull-up resistor
  attachInterrupt(5, doEncoder1, CHANGE);  // encoder pin on interrupt 0 - pin 5
  pinMode(encoder2PinA, INPUT);         // day cycle frequency rotary encoder
  digitalWrite(encoder2PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder2PinB, INPUT);
  digitalWrite(encoder2PinB, HIGH);       // turn on pull-up resistor
  attachInterrupt(0, doEncoder2, CHANGE);  // encoder pin on interrupt 0 - pin 2
  pinMode(encoder1switch, INPUT_PULLUP);
  pinMode(encoder2switch, INPUT_PULLUP);
  pinMode(PHdriftLED, OUTPUT);
  pinMode(WatertankLED, OUTPUT);
  initialise();
  startdip();
  startdripdry();
  readytostart();
}

void loop() {
  int encoder1switchval = digitalRead(encoder1switch);
  int encoder2switchval = digitalRead(encoder2switch);
  if ((encoder1switchval == 0)||(encoder2switchval == 0)) {
    if (encoderswitchflag == HIGH){
      encoderswitchflag = LOW;
      delay(500);
    }
    else {
      encoderswitchflag = HIGH;
      delay(500);
    }
  }
    // initialise fan
  analogWrite(enB, 255);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  int potval = analogRead(potPin);
    lcd.setCursor(0,0);
  lcd.print("Brightness: ");
  potval = map(potval, 0, 1019, 0, 100);
  lcd.print(potval);
  lcd.print("%  ");
  lcd.setCursor(0,1);
  lcd.print("Water Freq: ");
  if (waterfreqhrs < 47.9){
    lcd.print(waterfreqhrs, 1);
    lcd.print(" hrs");
  }
  else {
    lcd.print("OFF     ");
  }
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(SensorPin);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue;       //convert the millivolt into pH value
if (encoderswitchflag == HIGH) {
  int chk = DHT.read11(DHT11_PIN);
  if (DHT.temperature != -999.00) {
    lcd.setCursor(0,2);
    lcd.print("Temp:       ");
    Temperature = DHT.temperature;
    lcd.print(Temperature);
    lcd.print((char)223);
    lcd.print("c  ");
    }
  if (DHT.humidity != -999.00) {
    Humidity = DHT.humidity;
    lcd.setCursor(0,3);
    lcd.print("Humidity:   ");
    lcd.print(Humidity);    
    lcd.print("% ");
  }
}

  else {
    lcd.setCursor(0,2);
    lcd.print("Day/Night:  ");
    lcd.print(daycyclepercent);
    lcd.print("/");
    lcd.print(100-daycyclepercent);
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("PH: ");
    lcd.print(phValue, 1);
    lcd.print("        ");
}                  
long waterfreqmillis = waterfreqhrs*60*60*1000L;
unsigned long currentMillis1 = millis();
if ((unsigned long)(currentMillis1 - previousMillis1) >= waterfreqmillis/6) {
  dip();
  delay(4000);
  dripdry();
  previousMillis1 = millis();
  }
daycyclescaler = daycyclepercent*0.01;
long daycyclemillis = 86400000L *daycyclescaler;
unsigned long currentMillis2 = millis();
if (((unsigned long)(currentMillis2 - previousMillis2) >= daycyclemillis) && (daynight == false)) {
  digitalWrite(relay, LOW); //turn off light
  previousMillis3 = millis();
  daynight = true;
  Serial.println(daycyclemillis);
  }
nightcyclescaler = 1 - daycyclepercent*0.01;
long nightcyclemillis = 86400000L *nightcyclescaler;
unsigned long currentMillis3 = millis();
if(((unsigned long)(currentMillis3 - previousMillis3) >= (nightcyclemillis)) && (daynight == true)) {
  digitalWrite(relay, HIGH); //turn on lights
  previousMillis2 = millis();
  daynight = false;
  Serial.println(nightcyclemillis);
}
if ((phValue < 5.4) || (phValue > 7)) {
  digitalWrite(PHdriftLED, HIGH);
}
else{
  digitalWrite(PHdriftLED, LOW);
}
}
void dip(){ //submerge one tray
  hallState = digitalRead(hallPin);
  lcd.clear();
  lcd.setCursor(6,0);
  lcd.print("CAUTION!");
  lcd.setCursor(3,2);
  lcd.print("Dip in progress");
  while (hallState == HIGH){
    digitalWrite(motorrelay, HIGH); 
    hallState = digitalRead(hallPin);
  }
  digitalWrite(motorrelay, LOW); 
  Serial.println("dipped");
}
void startdip(){ //submerge one tray
  hallState = digitalRead(hallPin);
  lcd.clear();
  lcd.setCursor(6,0);
  lcd.print("CAUTION!");
  lcd.setCursor(0,2);
  lcd.print("Motor Initialisation");
  while (hallState == HIGH){
    digitalWrite(motorrelay, HIGH); 
    hallState = digitalRead(hallPin);
  }
  digitalWrite(motorrelay, LOW); 
  Serial.println("dipped");
  lcd.clear();
}
void dripdry(){ //drip dry one tray
  hallState = digitalRead(hallPin);
  while (hallState == LOW){
    digitalWrite(motorrelay, HIGH);
    hallState = digitalRead(hallPin);  
  }
  digitalWrite(motorrelay, HIGH);
  delay(2400);
  digitalWrite(motorrelay, LOW);
  Serial.println("dripdried");
  lcd.clear();
  
}
void startdripdry(){ //drip dry one tray
  hallState = digitalRead(hallPin);
  lcd.clear();
  lcd.setCursor(6,0);
  lcd.print("CAUTION!");
  lcd.setCursor(0,2);
  lcd.print("Position Calibration");
  while (hallState == LOW){
    digitalWrite(motorrelay, HIGH);
    hallState = digitalRead(hallPin);  
  lcd.clear();
  }
  digitalWrite(motorrelay, HIGH);
  delay(2400);
  digitalWrite(motorrelay, LOW);
  Serial.println("dripdried");
  lcd.clear();
}
void initialise(){ //initialisation process
  lcd.clear();
  lcd.print("Initialising...");
  delay(750);
  lcd.setCursor(0,1);
  lcd.print("Calibrating Sensors");
  delay(750);
  lcd.setCursor(0,2);
  lcd.print("Water Level Check");
  delay(750);
  lcd.setCursor(0,3);
  lcd.print("Starting Motors");
  delay(750);
  lcd.clear();
}
void readytostart() {
  lcd.clear();
  lcd.setCursor(8,1);
  lcd.print("READY");
  delay(1500);
  lcd.clear();
}
void doEncoder1() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
  encoder1Pos = constrain(encoder1Pos, 2.3, 960);
  waterfreqhrs = encoder1Pos/20;
}
void doEncoder2() {
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
    encoder2Pos++;
  } else {
    encoder2Pos--;
  }
  encoder2Pos = constrain(encoder2Pos, 0, 100);
  daycyclepercent = encoder2Pos;
}
 
