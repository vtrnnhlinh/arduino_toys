#include <Arduino.h>

#define T_OSC 100
#define TX_PIN 2
#define RX_PIN 3

short battery = 0;
short current = 0;
short currentPercent = 0;
short rpm = 0;
long faultCode = 0;
bool regen = false;
bool brake = false;
unsigned long lastTime;
unsigned long lastDuration = 0;
byte lastCrc = 0;
uint8_t data[12];
int bitIndex = -1;

void send_bit(uint8_t bit) {
  if (bit) {
    digitalWrite(TX_PIN, LOW);
    delayMicroseconds(32 * T_OSC);
    digitalWrite(TX_PIN, HIGH);
    delayMicroseconds(64 * T_OSC);
  } else {
    digitalWrite(TX_PIN, LOW);
    delayMicroseconds(64 * T_OSC);
    digitalWrite(TX_PIN, HIGH);
    delayMicroseconds(32 * T_OSC);
  }
}

void send_start_bit(void) {
  digitalWrite(TX_PIN, LOW);
  delayMicroseconds(992 * T_OSC);
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(32 * T_OSC);
}

void idle(void) {
  digitalWrite(TX_PIN, LOW);
  // delayMicroseconds(992 * T_OSC);
  // delayMicroseconds(9930 * T_OSC);
  //digitalWrite(TX_PIN, HIGH);
  //delayMicroseconds(33*T_OSC);
}

void send_byte(uint8_t data) {
  uint8_t mask = 0x80;

  for (uint8_t i = 0; i < 8; i++) {
    if (data & mask) {
      send_bit(1);
      // Serial.print(1);
    } else {
      send_bit(0);
      // Serial.print(0);
    }

    mask = mask >> 1;
  }
  // Serial.print("|");
}

void send_buff(uint8_t* buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    send_byte(buf[i]);
  }
  // Serial.println("");
}

void send_frame() {

  uint8_t tx_data[12];

  // Fill tx_data array
  tx_data[0] = 0x08;   // Device code (fixed)
  tx_data[1] = 0x61;   // Sequence code (fixed)
  tx_data[2] = 0x00;   // Example tx_data
  tx_data[3] = 0xFC;   // Example tx_data
  tx_data[4] = 0x01;   // Example tx_data
  tx_data[5] = 0x01;   // Example tx_data
  tx_data[6] = 0x00;   // Example tx_data
  tx_data[7] = 0x08;   // Example tx_data
  tx_data[8] = 0x01;   // Example tx_data
  tx_data[9] = 0x32;   // Example tx_data
  tx_data[10] = 0x00;  // Example tx_data

  // Calculate checksum
  byte crc = 0;
  for (int i = 0; i < 11; i++) {
    crc ^= tx_data[i];
  }
  //Serial.println("crc infunc" + String(tx_data[11]));
  tx_data[11] = crc;

  // send tx_data
  send_buff(tx_data, 12);
}

void sifChange() {
  int val = digitalRead(RX_PIN);
  unsigned long duration = micros() - lastTime;
  lastTime = micros();

  if (val == LOW) {
    if (lastDuration > 0) {
      bool bitComplete = false;
      float ratio = float(lastDuration) / float(duration);

      // Debug print statements
      // Serial.print("Duration: ");
      // Serial.print(duration);
      // Serial.print(",duration Last Duration: ");
      // Serial.print(lastDuration);
      // Serial.print(", Ratio: ");
      // Serial.println(ratio);

      if (round(lastDuration / duration) >= 31) {
        bitIndex = 0;
        Serial.println("start frame -------------------------------------------------------");
      } else if (ratio > 1.5) {
        // bit value 0
        bitClear(data[bitIndex / 8], 7 - (bitIndex % 8));
        bitComplete = true;
      } else if (1 / ratio > 1.5) {
        // bit value 1println
        bitSet(data[bitIndex / 8], 7 - (bitIndex % 8));
        bitComplete = true;
      } else {
        Serial.println(String(duration) + "-" + String(lastDuration));
      }
      if (bitIndex % 8 == 0 && bitIndex != 0) {
        Serial.println("---------------");
      }

      if (bitComplete) {
        Serial.println("Bit Complete");
        bitIndex++;
        Serial.println("bitIndex: " + String(bitIndex));
        if (bitIndex == 96) {
          bitIndex = 0;
          byte crc = 0;
          for (int i = 0; i < 11; i++) {
            crc ^= data[i];
          }
          if (crc != data[11]) {
            Serial.println("CRC FAILURE: " + String(crc) + "-" + String(data[11]));
          }
          if (crc == data[11]) {  // (crc == data[11] && crc != lastCrc)
            lastCrc = crc;
            for (int i = 0; i < 12; i++) {
              Serial.print(data[i], HEX);
              Serial.print(" ");
            }
            Serial.println();
            battery = data[9];
            current = data[6];
            currentPercent = data[10];
            rpm = ((data[7] << 8) + data[8]) * 1.91;
            brake = bitRead(data[4], 5);
            regen = bitRead(data[4], 3);

            Serial.print("Battery %: " + String(battery));
            Serial.print(" Current %: " + String(currentPercent));
            Serial.print(" Current A: " + String(current));
            Serial.print(" RPM: " + String(rpm));
            if (brake) Serial.print(" BRAKE");
            if (regen) Serial.print(" REGEN");
            Serial.println();
          }
        }
      }
    }
  }
  lastDuration = duration;
}
void setup() {
  Serial.begin(115200);
  pinMode(TX_PIN, OUTPUT);
  idle();
  pinMode(RX_PIN, INPUT);
  lastTime = micros();
  attachInterrupt(digitalPinToInterrupt(RX_PIN), sifChange, CHANGE);
}

void loop() {
  // idle();
  send_start_bit();
  send_frame();
  idle();
  delay(100);
  //delay(1000);
  // send_data(data);
}
//
