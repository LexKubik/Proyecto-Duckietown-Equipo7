int pinLedRed = 6;
int pinLedGreen = 5;
int pinLedBlue =3;
//int pinPote =A0;
//int pote =0;
//int pote_mv =0;

void setup() {
  // put your setup code here, to run once:
  pinMode(pinLedRed,OUTPUT);
  pinMode(pinLedGreen,OUTPUT);
  pinMode(pinLedBlue,OUTPUT);
 // Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
/*  pote = analogRead(A0);
  pote_mv = map(pote,0,1023,0,255);
  Serial.println(pote_mv);
  analogWrite(pinLed,pote_mv);
 
 // if(pote_mv>2500)
   // digitalWrite(pinLed, HIGH);
 // else digitalWrite(pinLed, LOW);*/
 digitalWrite(pinLedRed, HIGH);
 delay(5000);
 digitalWrite(pinLedRed, LOW);
 digitalWrite(pinLedGreen, HIGH);
 delay(5000);
 digitalWrite(pinLedGreen, LOW);
 digitalWrite(pinLedBlue, HIGH);
 delay(5000);
 digitalWrite(pinLedBlue, LOW);

}
