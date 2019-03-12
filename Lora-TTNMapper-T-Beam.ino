#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "config.h"
#include "gps.h"

// T-Beam specific hardware
#define BUILTIN_LED 21

// OTAA (true) or ABP (false)
#define USE_OTAA true
#define USE_SLEEP true

char s[32]; // used to sprintf for Serial output
uint8_t txBuffer[9];
gps gps;

// OTAA, see config.h for settings
#if defined(USE_OTAA) && USE_OTAA == true
void os_getArtEui(u1_t * buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t * buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t * buf) { memcpy_P(buf, APPKEY, 16); }
#else
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t * buf) {}
void os_getDevEui(u1_t * buf) {}
void os_getDevKey(u1_t * buf) {}
#endif

static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 90;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  90        /* Time ESP32 will go to sleep (in seconds) */

// variables to keep while sleeping and rebooting
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int packetCount = 0;
RTC_DATA_ATTR int otaaJoined = 0;
RTC_DATA_ATTR u1_t keep_nwkKey[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
RTC_DATA_ATTR u1_t keep_artKey[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
RTC_DATA_ATTR devaddr_t keep_devaddr = 0;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
        case 1  :
            Serial.println("Wakeup caused by external signal using RTC_IO");
            break;
        case 2  :
            Serial.println("Wakeup caused by external signal using RTC_CNTL");
            break;
        case 3  :
            Serial.println("Wakeup caused by timer");
            break;
        case 4  :
            Serial.println("Wakeup caused by touchpad");
            break;
        case 5  :
            Serial.println("Wakeup caused by ULP program");
            break;
        default :
            Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
            break;
    }
}


// Pin mapping
const lmic_pinmap lmic_pins = {
        .nss = 18,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = LMIC_UNUSED_PIN, // was "14,"
        .dio = {26, 33, 32},
};

void onEvent(ev_t ev) {
    switch (ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).

            Serial.println(F("Saving OTAA values after successfull join."));
            memcpy(keep_nwkKey, LMIC.nwkKey, 16);
            memcpy(keep_artKey, LMIC.artKey, 16);
            keep_devaddr = LMIC.devaddr;

            for(int i = 0; i < 16; i++) {
                Serial.print("0x");
                Serial.print(LMIC.nwkKey[i], HEX);
                Serial.print(" ");
            }
            Serial.println("");

            otaaJoined = 1;
            LMIC_setLinkCheckMode(0);

            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;

        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            digitalWrite(BUILTIN_LED, LOW);
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.println(F("Received Ack"));
            }
            if (LMIC.dataLen) {
                sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
                Serial.println(s);
                sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
                Serial.println(s);
            }

            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

            sleep();

            break;

        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t *j) {

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        if (gps.checkGpsFix()) {
            LMIC.seqnoUp = packetCount;
            LMIC.seqnoDn = packetCount;

            ++packetCount;

            // Prepare upstream data transmission at the next possible time.
            gps.buildPacket(txBuffer);
            LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
            Serial.println(F("Packet queued"));
            digitalWrite(BUILTIN_LED, HIGH);

        } else {
            // try again in 3 seconds
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
        }

    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    //*************************
    // Sleep
    //*************************
    //Increment boot number and print it every reboot
    //Serial.println("Boot number: " + String(packetCount));
    //Print the wakeup reason for ESP32
    #if defined(USE_SLEEP) && USE_SLEEP
    print_wakeup_reason();
    #endif


    //*************************
    // ESP32 and LMIC
    //*************************
    //Turn off WiFi and Bluetooth
    WiFi.mode(WIFI_OFF);
    btStop();
    gps.init();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // ABP or OTAA
    #if defined(USE_OTAA) && USE_OTAA == true
    Serial.println("Activation method: OTAA");
    #if defined(USE_SLEEP) && USE_SLEEP == true
    // only restore OTAA values when at least slept once.
    if (bootCount > 0) {
        Serial.println(F("Restoring OTAA session credentials from memory"));
        memcpy(LMIC.nwkKey, keep_nwkKey, 16);
        memcpy(LMIC.artKey, keep_artKey, 16);
        LMIC.devaddr = keep_devaddr;
    }
    #endif

    #else

    Serial.println("Activation method: ABP");
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF9, DR_SF9), BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF9, DR_SF9), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF9, DR_SF9), BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF9, DR_SF9), BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF9, DR_SF9), BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF9, DR_SF9), BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF9, DR_SF9), BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF9, DR_SF9), BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);      // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF9, 14);

    do_send(&sendjob);
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);


    //*************************
    // Sleep
    //*************************
    if (otaaJoined == 1) {
        sleep();
    } else {
        Serial.println("Not going to sleep because not joined the network yet (via OTAA)");
    }
}

void sleep() {
    bootCount++;
    #if defined(USE_SLEEP) && USE_SLEEP
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
    #endif
}

void loop() {
    os_runloop_once();
}
