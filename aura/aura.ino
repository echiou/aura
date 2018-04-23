#include <stdio.h>
#include <math.h>

#include <bluefruit.h>
#include <Adafruit_NeoPixel.h>

#define DEVICE_NAME               ("A U R A")
#define SECOND_MS                 (1000)

// Neopixel
#define NEOPIXEL_COUNT            (3)
#define NEOPIXEL_PIN              (16)
#define NEOPIXEL_PX_TYPE          NEO_GRB + NEO_KHZ800

#define PULSE_UPDATE_INTERVAL_MS  (5)
#define PULSE_LENGTH_MS           (2 * SECOND_MS)
#define PULSE_START_PERCENT       (0.7) // Chance for pulse to start every second.

typedef struct neopixel_pulse_state {
  uint8_t neopixel_num;
  uint8_t pulse_color[3];
  unsigned long pulse_length_ms;

  bool pulsing;
  unsigned long start_time;
  unsigned long last_rand_check;
} neopixel_pulse_state;

unsigned long neopixel_pulse_last_update;
neopixel_pulse_state neopixel_pulse_states[3];
void neopixel_pulse_start(Adafruit_NeoPixel* neopixels, struct neopixel_pulse_state* pulse_state);
void neopixel_pulse_update(Adafruit_NeoPixel* neopixels, struct neopixel_pulse_state* pulse_state);

Adafruit_NeoPixel neopixels = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEOPIXEL_PX_TYPE);

// Bluefruit
unsigned long bluefruit_last_dev_found;
void bluefruit_start_adv(void);
void bluefruit_scan_callback(ble_gap_evt_adv_report_t* report);

void setup()
{
  Serial.begin(115200);

  // Set up neopixels
  neopixels.begin();
  neopixels.show();

  // Set up neopixel pulses
  neopixel_pulse_last_update = millis();
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    neopixel_pulse_states[i] = {0};
    neopixel_pulse_states[i].neopixel_num = i;
    neopixel_pulse_states[i].pulse_color[0] = 255;
    neopixel_pulse_states[i].pulse_color[1] = 255;
    neopixel_pulse_states[i].pulse_color[2] = 255;
    neopixel_pulse_states[i].pulse_length_ms = PULSE_LENGTH_MS;
    neopixel_pulse_start(&neopixels, &neopixel_pulse_states[i]);
  }

  // Set up bluetooth
  Bluefruit.begin();
  Bluefruit.setName(DEVICE_NAME);
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.autoConnLed(false);

  bluefruit_last_dev_found = millis();
  /* Start Central Scanning
   * - Interval = 100 ms, window = 80 ms
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(bluefruit_scan_callback);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.start(0x3fff);

  // Set up and start advertising
  bluefruit_start_adv();
}

void loop()
{
  unsigned long cur_time = millis();
  if((cur_time - neopixel_pulse_last_update) > PULSE_UPDATE_INTERVAL_MS)
  {
    neopixel_pulse_last_update = cur_time;
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
      if (neopixel_pulse_states[i].pulsing) {
        neopixel_pulse_update(&neopixels, &neopixel_pulse_states[i]);
      } else {
        if (cur_time - neopixel_pulse_states[i].last_rand_check > SECOND_MS) {
          neopixel_pulse_states[i].last_rand_check = cur_time;
          if (random(0, 100) < PULSE_START_PERCENT * 100) {
            uint8_t rand_color = random(0, 255);
            neopixel_pulse_states[i].pulse_color[0] = 0;
            neopixel_pulse_states[i].pulse_color[1] = rand_color;
            neopixel_pulse_states[i].pulse_color[2] = 255 - rand_color;
            neopixel_pulse_states[i].pulse_length_ms = PULSE_LENGTH_MS;
            neopixel_pulse_start(&neopixels, &neopixel_pulse_states[i]);
          }
        }
      }
    }
    neopixels.show();
  }
        
  // Check lastTimeFound and lastRSSI, depending on each change colors from red -> blue
}

// Bluefruit functions

void bluefruit_start_adv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Interval:  fast mode = 20 ms, slow mode = 1285 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   */
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(1);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0x3fff);          // 0x3fff = about 4 hrs.
}

void bluefruit_scan_callback(ble_gap_evt_adv_report_t* report)
{ 
  uint8_t buffer[32];
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    if (String((char *)buffer).indexOf(DEVICE_NAME) >= 0) {
      unsigned long time_found = millis();
      unsigned long time_diff = time_found - bluefruit_last_dev_found;
      bluefruit_last_dev_found = time_found;
      // Update lastTimeFound, RSSI
      Serial.printf("Found %s with RSSI: %d, last seen %lu ms ago. Time is %lu\r\n", DEVICE_NAME, report->rssi, time_diff, time_found);
    }
  }
}

void neopixel_pulse_start(Adafruit_NeoPixel* neopixels, struct neopixel_pulse_state* pulse_state)
{
  neopixels->setPixelColor(pulse_state->neopixel_num, neopixels->Color(0, 0, 0));
  pulse_state->start_time = millis();
  pulse_state->pulsing = true;
}

void neopixel_pulse_update(Adafruit_NeoPixel* neopixels, struct neopixel_pulse_state* pulse_state)
{
  unsigned long cur_time = millis();
  if (pulse_state->pulsing == false || pulse_state->start_time + pulse_state->pulse_length_ms < cur_time) {
    neopixels->setPixelColor(pulse_state->neopixel_num, neopixels->Color(0, 0, 0));
    pulse_state->pulsing = false;
    pulse_state->last_rand_check = cur_time + random(-500, 500);
    return;
  }

  float percent;
  unsigned long time_delta = cur_time - pulse_state->start_time;
  unsigned long half_pulse_length = pulse_state->pulse_length_ms / 2;

  float breath_percent = (sin((float)time_delta / pulse_state->pulse_length_ms * PI));
  uint8_t new_r = breath_percent * pulse_state->pulse_color[0];
  uint8_t new_g = breath_percent * pulse_state->pulse_color[1];
  uint8_t new_b = breath_percent * pulse_state->pulse_color[2];
  neopixels->setPixelColor(pulse_state->neopixel_num, neopixels->Color(new_r, new_g, new_b));
}
