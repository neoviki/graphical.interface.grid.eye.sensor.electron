void setup() {
  Serial.begin(9600);
}

void loop() {
  int i, r;
  String s = "GRID_EYE#";
  for(i=0; i<64; i++){
    r = random(0, 256); /*generate between 0 to 255*/
    s += String(r);
    s += "#";
  }
  Serial.println("");
  Serial.println(s);
  Serial.println("");
  delay(1000);
}
