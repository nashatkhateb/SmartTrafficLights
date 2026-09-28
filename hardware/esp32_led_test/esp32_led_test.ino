/*
 * Standalone wiring test for the 3-LED traffic-light module (the physically verified sketch, kept
 * unchanged below). G -> GPIO25, Y -> GPIO33, R -> GPIO32, GND -> GND; HIGH = lamp ON.
 *
 * It runs its OWN fixed timing (red 2 s, red+yellow 1 s, green 3 s, yellow 2 s) and does
 * not talk to the PC. Use it only to check the wiring; for the project flash
 * hardware/esp32_smart_traffic/esp32_smart_traffic.ino, which mirrors the simulator.
 */
const int GREEN_PIN  = 25;
const int YELLOW_PIN = 33;
const int RED_PIN    = 32;

void allOff() {
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(RED_PIN, LOW);
}

void setRed() {
  allOff();
  digitalWrite(RED_PIN, HIGH);
}

void setRedYellow() {
  allOff();
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(YELLOW_PIN, HIGH);
}

void setGreen() {
  allOff();
  digitalWrite(GREEN_PIN, HIGH);
}

void setYellow() {
  allOff();
  digitalWrite(YELLOW_PIN, HIGH);
}

void setup() {
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);

  setRed();
}

void loop() {
  setRed();
  delay(2000);

  setRedYellow();
  delay(1000);

  setGreen();
  delay(3000);

  setYellow();
  delay(2000);
}
