#include "src/dw1000/DW1000.h"
#include "src/uwb_led/uwb_led.h"
#include <SPI.h>

////////////////////////////
#define NUM_ANCHORS 6
#define OUR_ID 3
LED uwb_leds;
////////////////////////////

#define DELAY_TIME_US 3331
#define TIMEOUT_US 20000

#define DELAY_BETWEEN_RANGINGS_MS 1

#define THIS_IS_ANCHOR (OUR_ID != 0)
#define THIS_IS_TAG (OUR_ID == 0)

const uint8_t PIN_RST = 0; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = 7;  // spi select pin

#define LEN_DATA 256
byte data[LEN_DATA];

enum msg_t { PLL,
             RES,
             FNL,
             REP };
enum state_t { WAY1,
               WAY2,
               BROADCAST } state;

class Ranging {
public:
  DW1000Time T0;
  DW1000Time T1;
  DW1000Time T2;
  DW1000Time T3;
  DW1000Time T4;
  DW1000Time T5;
  float lastComputedRange;

  void computeRange();

  float getLastComputedRange() {
    return this->lastComputedRange;
  }
};

Ranging ranging;
uint8_t current_anchor = 1;
volatile bool received;
volatile bool sent;
volatile bool timerEnabled = false;
unsigned long timerTimeout;
String output;
DW1000Time Tr;

void Ranging::computeRange() {
  DW1000Time round1 = (T3 - T0).wrap();
  DW1000Time reply1 = (T2 - T1).wrap();
  DW1000Time round2 = (T5 - T2).wrap();
  DW1000Time reply2 = (T4 - T3).wrap();

  DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
  float range = tof.getAsMeters();
  if(range < 1000 || range >= -10) {
    this->lastComputedRange = range;
  } else {
    this->lastComputedRange = 0;
  }
}

void setup() {
  received = false;
  sent = false;
  Serial.begin(115200);
  delay(2000);
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(OUR_ID);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);

#if THIS_IS_TAG == 1
  Serial.println("TAG DEVICE");
  DW1000.commitConfiguration(16470);
#else
  Serial.println("ANCHOR DEVICE");
  DW1000.commitConfiguration(16500);
#endif
  Serial.println(F("Committed configuration ..."));

  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);

  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();

  output.reserve(200);
  output = "";

  uwb_leds.blink(LED::BOTH, 100, 2);

  state = WAY1;
#if THIS_IS_TAG == 1
  doRequest();
  state = WAY2;
#endif
}

void handleSent() {
  sent = true;
}

void handleReceived() {
  received = true;
}

void parseReceivedTag() {
  unsigned int len = DW1000.getDataLength();
  DW1000.getData(data, len);
  DW1000.getReceiveTimestamp(Tr);

  if(len != 13 && len != 6) {
    Serial.println("Received incomplete message, error.");
    return;
  }

  byte MSG_TYPE = data[0];
  if(MSG_TYPE == RES && state == WAY2) {
    timerEnabled = false;
    doRequest();
    state = BROADCAST;
  } else if(MSG_TYPE == REP && state == BROADCAST) {
    timerEnabled = false;
    state = WAY1;
    current_anchor++;
    if(current_anchor > NUM_ANCHORS) current_anchor = 1;
    doRequest();
    state = WAY2;
  }
}

void parseReceivedAnchor() {
  unsigned int len = DW1000.getDataLength();
  DW1000.getData(data, len);
  DW1000.getReceiveTimestamp(Tr);

  if(len != 13 && len != 6) {
    Serial.println("Received incomplete message, error.");
    return;
  }

  byte MSG_TYPE = data[0];
  if(MSG_TYPE == REP) {
    float *val = (float *)(data + 2);
    Serial.println(String(data[1]) + "@" + String(*val, 4));
  } else if(MSG_TYPE == PLL && data[2] == OUR_ID && state == WAY1) {
    timerEnabled = false;
    ranging.T0.setTimestamp(data + 8);
    ranging.T1 = Tr;
    doResponse();
    state = WAY2;
  } else if(MSG_TYPE == FNL && data[2] == OUR_ID && state == WAY2) {
    timerEnabled = false;
    ranging.T3.setTimestamp(data + 3);
    ranging.T4.setTimestamp(data + 8);
    ranging.T5 = Tr;
    state = BROADCAST;
    doReport();
    state = WAY1;
  }
}

void doRequest() {
  // Type
  if(state == WAY1)
    data[0] = PLL;
  else
    data[0] = FNL;
  // Sender
  data[1] = OUR_ID;
  // Receiver
  data[2] = current_anchor;
  // Time received
  Tr.getTimestamp(data + 3);
  // Transmit
  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000Time deltaTime = DW1000Time(DELAY_TIME_US, DW1000Time::MICROSECONDS);
  DW1000Time Ts = DW1000.setDelay(deltaTime);
  Ts.getTimestamp(data + 8);
  DW1000.setData(data, 13);
  DW1000.startTransmit();

  while(!sent);
  sent = false;

  // Reset timer
  timerTimeout = micros();
  timerEnabled = true;
}

void doResponse() {
  // Type
  data[0] = RES;
  // Sender
  data[1] = OUR_ID;
  // Receiver
  data[2] = 0;
  // Tr not required, left as it is for efficiency
  // Transmit with time sent
  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000Time deltaTime = DW1000Time(DELAY_TIME_US, DW1000Time::MICROSECONDS);
  DW1000Time Ts = DW1000.setDelay(deltaTime);
  ranging.T2 = Ts;
  DW1000.setData(data, 13);
  DW1000.startTransmit();

  while(!sent);
  sent = false;
  received = false;

  // Reset timer
  timerTimeout = micros();
  timerEnabled = true;
}

void doReport() {
  ranging.computeRange();
  data[0] = REP;
  data[1] = OUR_ID;
  memcpy(data + 2, &(ranging.lastComputedRange), 4);

  float *val = (float *)(data + 2);
  Serial.println(String(data[1]) + "@" + String(*val, 4));

  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000Time deltaTime = DW1000Time(DELAY_TIME_US, DW1000Time::MICROSECONDS);
  DW1000Time Ts = DW1000.setDelay(deltaTime);
  DW1000.setData(data, 6);
  DW1000.startTransmit();

  while(!sent);
  sent = false;
}

void loop() {
#if THIS_IS_TAG == 1
  if(received) {
    parseReceivedTag();
    received = false;
  } else if(timerEnabled && micros() - timerTimeout > TIMEOUT_US) {
    timerEnabled = false;
    state = WAY1;
    current_anchor++;
    if(current_anchor > NUM_ANCHORS) current_anchor = 1;
    doRequest();
    state = WAY2;
  }
#else // THIS_IS_ANCHOR
  if(received) {
    uwb_leds.set(LED::GREEN, HIGH);
    parseReceivedAnchor();
    received = false;
    uwb_leds.set(LED::GREEN, LOW);
  } else if(timerEnabled && micros() - timerTimeout > TIMEOUT_US) {
    timerEnabled = false;
    uwb_leds.set(LED::RED, HIGH);
    state = WAY1;
    uwb_leds.set(LED::RED, LOW);
  }
#endif
}
