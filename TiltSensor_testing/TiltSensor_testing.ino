const int statusLED = 13;
const int switchTilt = 17;

int val = 0;

void setup()
{
  Serial.begin(115200);
  
  pinMode (statusLED, OUTPUT);
  pinMode (switchTilt, INPUT);
}

bool inversed()
{
  val = digitalRead(switchTilt);

  if (val == HIGH)
    return 0;
  else
    return 1;
}

void loop() {
  val = inversed();
  
  if (val == 0) {
    digitalWrite(statusLED, HIGH);
    Serial.print(val);
    Serial.print(' ');
    Serial.println("DOWN");
  }
  else {
    digitalWrite(statusLED, LOW);
    Serial.print(val);  
    Serial.print(' ');
    Serial.println("UP");
  }
}
