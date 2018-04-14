#include <bluefruit.h>
#include <stdio.h>

#define DEVICE_NAME "A U R A"

// Static lastTimeFound, lastRSSI
unsigned long lastTimeFound;

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello I am setting up");

  Bluefruit.begin();
  Bluefruit.setName(DEVICE_NAME);
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.autoConnLed(false);

  lastTimeFound = millis();
  /* Start Central Scanning
   * - Interval = 100 ms, window = 80 ms
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setInterval(2056, 2056); // in unit of 0.625 ms
  Bluefruit.Scanner.start(0);

  // Set up and start advertising
  startAdv();
}

void startAdv(void)
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
  Bluefruit.Advertising.setInterval(32, 2056);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(1);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop()
{
  // Check lastTimeFound and lastRSSI, depending on each change colors from red -> blue
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{ 
  unsigned long timeFound = millis();
  unsigned long timeDiff = timeFound - lastTimeFound;
  lastTimeFound = timeFound;
  uint8_t buffer[32];
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    if (String((char *)buffer).indexOf(DEVICE_NAME) >= 0) {
      // Update lastTimeFound, RSSI
      Serial.printf("Found %s with RSSI: %d, last seen %lu ms ago\r\n", DEVICE_NAME, report->rssi, timeDiff);
    }
  }
}

