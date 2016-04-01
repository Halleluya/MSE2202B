const unsigned leftClawHall = A3;
const unsigned rightClawHall = A4;
unsigned long sensorValue = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(rightClawHall,INPUT);
  pinMode(leftClawHall,INPUT);
}

void loop() {
/*
  Serial.print(analogRead(leftClawHall));
  Serial.print("     ");
  Serial.println(analogRead(rightClawHall));
  */
  
  
  Serial.print("LeftClaw: ");

  if (analogRead(leftClawHall) < 504 || analogRead(leftClawHall) > 507)
  {
  Serial.print("Detected!");
  }
  else{Serial.print("-------");}

  
  Serial.print("      RightClaw:  ");
  if (analogRead(rightClawHall) < 505 || analogRead(rightClawHall) > 509)
  {
    Serial.println("Detected!");
  }
  else{Serial.println("-------");}
}

