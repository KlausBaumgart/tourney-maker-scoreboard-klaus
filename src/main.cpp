#include <Arduino.h>
#include <ostream>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);ghhgh
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("sdf");
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}