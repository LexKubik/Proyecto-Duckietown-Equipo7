int LED_rojo=4;
int LED_verde=2;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_rojo, OUTPUT);
  pinMode(LED_verde, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_rojo, HIGH);
  delay(10000);
  digitalWrite(LED_rojo, LOW);
  digitalWrite(LED_verde, HIGH);
  delay(10000);
  digitalWrite(LED_verde, LOW);
  
}
