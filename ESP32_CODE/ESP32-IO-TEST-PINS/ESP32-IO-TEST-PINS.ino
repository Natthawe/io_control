// ------- Input pins -------
const uint8_t INPUT_PINS[] = {22, 4, 15, 21, 36, 39, 34, 35};
const bool    USE_PULLUP[] = {true, true, true, true, true, true, true, true};
const int     NUM_IO       = sizeof(INPUT_PINS) / sizeof(INPUT_PINS[0]);

// ------- Output pins (1:1 กับ input) -------
const uint8_t OUTPUT_PINS[] = {13, 12, 14, 27, 32, 33, 25, 26};

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_IO; ++i) {
    pinMode(INPUT_PINS[i], USE_PULLUP[i] ? INPUT_PULLUP : INPUT);
  }

  for (int i = 0; i < NUM_IO; ++i) {
    pinMode(OUTPUT_PINS[i], OUTPUT);
    digitalWrite(OUTPUT_PINS[i], LOW);
  }

}

void loop() {
  for (int i = 0; i < NUM_IO; ++i) {
    int val = digitalRead(INPUT_PINS[i]);

    // use pullup: active LOW (0)
    // not use pullup: active HIGH (1)
    bool active = USE_PULLUP[i] ? (val == LOW) : (val == HIGH);

    digitalWrite(OUTPUT_PINS[i], active ? HIGH : LOW);

    Serial.printf("%d%s", val, (i == NUM_IO - 1) ? "\n" : " | ");
  }

  delay(30);
}
