// Offene Punkte
// Verzögerte aktivierung
// Sensoren bleiben aktiv und setzen sich erts nach ca. 10 Sekunden zurück
// deepsleep und lightsleep
// Beschreibung I2C Anschlüsse
// VIN = rt = 5V
// SHDN = ge = 5V
// VREF_IN = ws =3V3
// SCL_IN = bl = P22
// SDA_IN = gn = P21
// GND = sw = GND


#include <scoreboard.h>
#include <Wire.h>

TourneyMakerScoreboard *scoreboard = NULL;


String padToLength(const String& input, char padChar, int numModules) {
    if (input.length() >= numModules) return input;
    return padToLength(String(padChar) + input, padChar, numModules);
}

void sendI2CData(const String &rawContent, int numModules, uint8_t i2c_address) {
    String content = padToLength(rawContent, '0', numModules);

    Serial.println("sending data '" + content + "'...");

    // Define buffer size as 5 header bytes + numModules characters + 3 for checksum and ETX
    int bufferSize = 5 + numModules + 3;
    uint8_t buffer[bufferSize];
 
    // Fill header bytes
    buffer[0] = 0x02;                      // STX (Start of Text)
    buffer[1] = ((numModules + 2) >> 8);   // Length High byte
    buffer[2] = (numModules + 2) & 0xFF;   // Length Low byte
    buffer[3] = numModules;                // Number of modules
    buffer[4] = 0x01;                      // Mode (Segment mode by default)
 
    // Fill content in ASCII values, adding ASCII 32 (space) if content is shorter than numModules
    int contentSize = content.length();
    for (int i = 0; i < numModules; i++) {
        buffer[5 + i] = (i < contentSize) ? content[i] : 32;  // ASCII 32 for space
    }
 
    // Compute checksum from buffer[3] to buffer[numModules + 4]
    uint16_t checksum = 0;
    for (int i = 3; i < 5 + numModules; i++) {
        checksum += buffer[i];
    }
 
    // Add checksum and end character to buffer
    buffer[5 + numModules] = (checksum >> 8);        // Checksum High byte
    buffer[6 + numModules] = checksum & 0xFF;        // Checksum Low byte
    buffer[7 + numModules] = 0x03;                   // ETX (End of Text)
 
    // Send data over I2C
    Wire.begin();
    Wire.setClock(10000);
    Wire.beginTransmission(i2c_address);
    for (int i = 0; i < bufferSize; i++) {
        Wire.write(buffer[i]);
        if (i != 0) Serial.print("-");
        Serial.print(buffer[i]);
        delay(10);
    }
    Serial.println();
    Wire.endTransmission();
}


class MyScoreReceivedCallback : public ScoreboardChangedCallback
{
  void onScoreReceived(uint8_t score1, uint8_t score2)
  {
    Serial.println("score received in callback " + String(score1) + ":" + String(score2));
  }

  void onColorReceived(uint32_t color1, uint32_t color2) {
    Serial.println("color received in callback " + String(color1) + ":" + String(color2));
  }
};


#define i2cAddress 8        // Define the I2C address of the device


// Pins für die Buttons
const int btn1 = 25;
const int btn2 = 26;
const int btn3 = 35;
const int btn4 = 34;
const int btn5 = 32;

// Variablen für Zahlen
int leftNumber = 0;
int rightNumber = 0;
int longPress = 1000;
int repeatInterval =150;     // Wiederholrate bei gehaltenem Button (in ms)
int signalLED = 150;
const unsigned long activationDelay = 100;  // Zeit, bevor der Sensor als "gedrückt" gilt


// Variablen für Ausgänge
const int LED_PIN = 5; 


// --- Zustand der Buttons
volatile bool buttonPressed[5] = {false, false, false, false, false};
volatile unsigned long pressTime[5] = {0, 0, 0, 0, 0};

bool isHeld[5] = {false, false, false, false, false};
unsigned long lastActionTime[5] = {0, 0, 0, 0, 0};
bool actionDone[5] = {false, false, false, false, false};  // damit wir nicht mehrfach zählen beim kurzen Druck


void IRAM_ATTR handleButton1() { buttonPressed[0] = true; pressTime[0] = millis(); }
void IRAM_ATTR handleButton2() { buttonPressed[1] = true; pressTime[1] = millis(); }
void IRAM_ATTR handleButton3() { buttonPressed[2] = true; pressTime[2] = millis(); }
void IRAM_ATTR handleButton4() { buttonPressed[3] = true; pressTime[3] = millis(); }
void IRAM_ATTR handleButton5() { buttonPressed[4] = true; pressTime[4] = millis(); }

void setup() {
  Serial.begin(9600);

  scoreboard = TourneyMakerScoreboard::setup("Papa");

  scoreboard->scoreboardChangedCallback = new MyScoreReceivedCallback();


  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
  pinMode(btn5, INPUT_PULLUP);

  // Interrupts auf FALLING (Button gedrückt)
  attachInterrupt(digitalPinToInterrupt(btn1), handleButton1, RISING);
  attachInterrupt(digitalPinToInterrupt(btn2), handleButton2, RISING);
  attachInterrupt(digitalPinToInterrupt(btn3), handleButton3, RISING);
  attachInterrupt(digitalPinToInterrupt(btn4), handleButton4, RISING);
  attachInterrupt(digitalPinToInterrupt(btn5), handleButton5, RISING);

// Port für Ausgänge
  pinMode(LED_PIN, OUTPUT);

}

// --- Hilfsfunktion: LED blinken
void blinkLED() {
  digitalWrite(LED_PIN, HIGH);
  delay(signalLED);
  digitalWrite(LED_PIN, LOW);
}

// --- einmalige Aktion pro Button
void handleButtonAction(int index, int &value, int delta, const char* label) {
  value += delta;
  Serial.print(label);
  sendI2CData(String((leftNumber * 10 + rightNumber)),2,i2cAddress);
  scoreboard->setScore(leftNumber, rightNumber);
  Serial.print(" Left: "); Serial.print(leftNumber);
  Serial.print(" Right: "); Serial.println(rightNumber);
  Serial.println( String((leftNumber + rightNumber)));
  Serial.println( String((leftNumber * 10 + rightNumber)));
  blinkLED();
  lastActionTime[index] = millis();
  actionDone[index] = true;
}


void loop() {
  unsigned long currentTime = millis();


  // Serial.print("loop");
  // sleep(5);
  // Serial.print("test");
  // scoreboard->setScore(5, 0);

    // Buttons 1–4: zählen
  for (int i = 0; i < 4; i++) {
    int pin = (i == 0) ? btn1 : (i == 1) ? btn2 : (i == 2) ? btn3 : btn4;
    bool isPressed = digitalRead(pin) == HIGH;

    if (buttonPressed[i]) {
      // Taste wird gehalten
      if (isPressed) {
        unsigned long heldTime = currentTime - pressTime[i];

        if (heldTime >= activationDelay) {
          // Jetzt ist die Verzögerung vorbei: Taste gilt als "wirklich gedrückt"

          // --- einmalige Aktion bei Kurz-Druck
          if (!actionDone[i] && heldTime < longPress) {
            if (i == 0) handleButtonAction(i, rightNumber, +1, "Btn1:");
            if (i == 1) handleButtonAction(i, rightNumber, -1, "Btn2:");
            if (i == 2) handleButtonAction(i, leftNumber, +1, "Btn3:");
            if (i == 3) handleButtonAction(i, leftNumber, -1, "Btn4:");
            actionDone[i] = true;
          }

          // --- Wiederholung bei Lang-Druck
          if (heldTime >= longPress && (currentTime - lastActionTime[i] >= repeatInterval)) {
            if (i == 0) handleButtonAction(i, rightNumber, +1, "Btn1:");
            if (i == 1) handleButtonAction(i, rightNumber, -1, "Btn2:");
            if (i == 2) handleButtonAction(i, leftNumber, +1, "Btn3:");
            if (i == 3) handleButtonAction(i, leftNumber, -1, "Btn4:");
          }
        }   
     }

    // Taste wurde losgelassen
      if (!isPressed) {
        buttonPressed[i] = false;
        actionDone[i] = false;
      }
    } 
  }


  if (buttonPressed[4]) {
    bool isPressed = digitalRead(btn5) == HIGH;

    if (isPressed) {
      unsigned long heldTime = currentTime - pressTime[4];

      if (heldTime >= activationDelay && heldTime > 1000 && !actionDone[4]) {
        leftNumber = 0;
        rightNumber = 0;
        Serial.println("Reset beide Zahlen auf 0");
        scoreboard->setScore(leftNumber, rightNumber);
        sendI2CData(String(rightNumber),2,i2cAddress);
        blinkLED();
        actionDone[4] = true;
      }
    }

    if (!isPressed) {
      buttonPressed[4] = false;
      actionDone[4] = false;
    }
  }

}