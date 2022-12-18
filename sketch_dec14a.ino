int rojo1=2;
int rojo2=3;
int verde1=4;
int verde2=5;
void setup() {
  // put your setup code here, to run once:
  pinMode(rojo1, OUTPUT);
  pinMode(rojo2, OUTPUT);
  pinMode(verde1, OUTPUT);
  pinMode(verde2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(rojo1, HIGH);
  digitalWrite(rojo2, HIGH);
  delay(10000);
  digitalWrite(rojo1, LOW);
  digitalWrite(rojo2, LOW);
  digitalWrite(verde1, HIGH);
  digitalWrite(verde2, HIGH);
  delay(10000);
  digitalWrite(verde1, LOW);
  digitalWrite(verde2, LOW);
  
}
