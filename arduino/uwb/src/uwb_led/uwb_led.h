const uint8_t PIN_LED_G = 1;
const uint8_t PIN_LED_R = 3;

struct LED {
  enum : byte {
    BOTH = 0,
    GREEN = PIN_LED_G,
    RED = PIN_LED_R
  };

  LED() {
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
  }

  void set(byte sel, bool on_off) {
    if(sel == BOTH) {
      digitalWrite(GREEN, on_off);
      digitalWrite(RED, on_off);
    } else {
      digitalWrite(sel, on_off);
    }
  }

  void blink(byte sel, int wait) {
    blink(sel, wait, 1);
  }

  void blink(byte sel, int wait, int repeat) {
    for(int i = 0; i < repeat; i++) {
      set(sel, HIGH);
      delay(wait);
      set(sel, LOW);
      delay(wait);
    }
  }
};
