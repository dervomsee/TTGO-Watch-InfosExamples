/**
 * @file lilygo main.cpp
 *
 */
/*********************
 *      INCLUDES
 *********************/
#include "config.h"
#include <WiFi.h>
#include <nmeaparser.h>

NMEAParser parser;

#define MAX30105_ADDRESS 0x57
#define PCF8563_ADDRESS 0x51
#define BMA423_ADDRESS 0x19
#define AXP202_ADDRESS 0x35
#define FT6206_ADDRESS 0X38
#define MPU6050_ADDRESS 0x68
#define MPR121_ADDRESS 0x5A
#define DRV2605_ADDRESS 0x5A

/**********************
 *  STATIC VARIABLES
 **********************/
static TTGOClass *ttgo = nullptr;
static TFT_eSPI *tft = nullptr;
static AXP20X_Class *pmu = nullptr;
static TinyGPSPlus *gps;

static bool find_pwr, find_bma, find_pcf, find_mpu, find_max, find_tp, find_drv;
static bool pmu_irq;

#define BUFFER_SIZE (2 * TFT_WIDTH)
uint8_t buffer[BUFFER_SIZE] = {0};
uint16_t *ptr = nullptr;
uint16_t prevX[TFT_WIDTH];
uint16_t prevY[TFT_WIDTH];

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void display_info(void);
static void devScan(TwoWire &w);
static void resultOutput(String str, bool ret);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void setup()
{
    Serial.begin(115200);

    setCpuFrequencyMhz(240);

    ttgo = TTGOClass::getWatch();

    ttgo->begin();

    ttgo->openBL();

    tft = ttgo->tft;

    gps = ttgo->gps;

    pmu = ttgo->power;

    //! i2c device scan
    devScan(Wire);
    devScan(Wire1);

    if (find_pwr)
    {
        Serial.println("OK");
        pinMode(AXP202_INT, INPUT);
        attachInterrupt(
            AXP202_INT, [] {
                pmu_irq = true;
            },
            FALLING);

        ttgo->power->adc1Enable(AXP202_BATT_CUR_ADC1 | AXP202_BATT_VOL_ADC1 |
                                    AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1,
                                true);
        ttgo->power->enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ |
                                   AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ |
                                   AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ,
                               true);
        ttgo->power->clearIRQ();
    }
    tft->fillScreen(TFT_BLACK);

    tft->fillScreen(TFT_BLACK);
    tft->setTextColor(TFT_WHITE);
    tft->setCursor(0, 0);

    resultOutput("Find AXP202 Chip", find_pwr);

    ttgo->trunOnGPS();

    ttgo->gps_begin();

    gps = ttgo->gps;
}

char c_buffer[200];
void loop()
{

    if (pmu_irq)
    {
        pmu_irq = false;
        ttgo->power->readIRQ();

        if (ttgo->power->isPEKLongtPressIRQ())
        {
            Serial.println("isPEKLongtPressIRQ");
        }
        if (ttgo->power->isPEKShortPressIRQ())
        {
            Serial.println("isPEKShortPressIRQ");

            //the following messages can be used for writing NMEA messages to the L76K GPS receiver

            //char array for NMEA message
            char msg[100] = {0};

            //enable gps+baidu
            //sprintf(msg, "$PCAS04,3*%X\r\n", (int)parser.generate_checksum("$PCAS04,3"));

            //enable GPS + BeiDou + GLONASS
            sprintf(msg, "$PCAS04,7*%X\r\n", (int)parser.generate_checksum("$PCAS04,7"));
            
            //PCAS03
            //sprintf(msg, "$PCAS03,1,1,1,1,1,1,1,1,0,0,,,0,0*%X\r\n", (int)parser.generate_checksum("$PCAS03,1,1,1,1,1,1,1,1,0,0,,,0,0"));            

            //hardreset
            //sprintf(msg, "$PCAS10,3*%X\r\n", (int)parser.generate_checksum("$PCAS10,3"));

            Serial.println(msg);
            ttgo->hwSerial->write(msg, 14);
        }
        ttgo->power->clearIRQ();
    }

    while (ttgo->hwSerial->available())
    {
        //int r = ttgo->hwSerial->read();
        String s = ttgo->hwSerial->readStringUntil('\n');
        if (s.length() > 0)
            s = s.substring(0, s.length() - 1);
        //Serial.println(s);
        if (parser.dispatch(s))
        {
            // gest the last parsed sentence type
            switch (parser.getLastProcessedType())
            {
            // is it a GPRMC sentence?
            case NMEAParser::TYPE_GNRMC:
                //show some of the sentence's data
                //Serial.println("\nFound GNRMC sentence");
                //sprintf(c_buffer, " I'm at %s %c / %s %c", parser.last_gprmc.latitude, parser.last_gprmc.north_south_indicator, parser.last_gprmc.longitude, parser.last_gprmc.east_west_indicator);
                //Serial.println(c_buffer);
                break;
            case NMEAParser::TYPE_GPRMC:
                //show some of the sentence's data
                Serial.println("\nFound GPRMC sentence");
                sprintf(c_buffer, " I'm at %s %c / %s %c", parser.last_gprmc.latitude, parser.last_gprmc.north_south_indicator, parser.last_gprmc.longitude, parser.last_gprmc.east_west_indicator);
                Serial.println(c_buffer);
                break;
            case NMEAParser::TYPE_GPGSV:
                //show some of the sentence's data
                //GPGSV sentences most of the time comes in pair, sometimes more, as each of these can give info about 4 sats at a time.
                //So let's display this header only once
                if (parser.last_gpgsv.message_idx == 1)
                {
                    //Serial.println("\nFound GPGSV sentence");
                    //sprintf(c_buffer, " GPS can see %d satellites:", parser.last_gpgsv.sats_in_view);
                    //Serial.println(c_buffer);
                }
                /*
                sprintf(c_buffer, " Satellite ID=%X, elevation=%d, azimuth=%d, signal/noise ratio: %d", parser.last_gpgsv.sat1_id, parser.last_gpgsv.sat1_elevation, parser.last_gpgsv.sat1_azimuth, parser.last_gpgsv.sat1_snr);
                Serial.println(c_buffer);
                sprintf(c_buffer, " Satellite ID=%X, elevation=%d, azimuth=%d, signal/noise ratio: %d", parser.last_gpgsv.sat2_id, parser.last_gpgsv.sat2_elevation, parser.last_gpgsv.sat2_azimuth, parser.last_gpgsv.sat2_snr);
                Serial.println(c_buffer);
                sprintf(c_buffer, " Satellite ID=%X, elevation=%d, azimuth=%d, signal/noise ratio: %d", parser.last_gpgsv.sat3_id, parser.last_gpgsv.sat3_elevation, parser.last_gpgsv.sat3_azimuth, parser.last_gpgsv.sat3_snr);
                Serial.println(c_buffer);
                sprintf(c_buffer, " Satellite ID=%X, elevation=%d, azimuth=%d, signal/noise ratio: %d", parser.last_gpgsv.sat4_id, parser.last_gpgsv.sat4_elevation, parser.last_gpgsv.sat4_azimuth, parser.last_gpgsv.sat4_snr);
                Serial.println(c_buffer);
                */
                break;
            case NMEAParser::TYPE_GPTXT:
                sprintf(c_buffer, "number_of_messages: %d, sentence_number%d, text_identifier: %d, message: %s", parser.last_gptxt.number_of_messages, parser.last_gptxt.sentence_number, parser.last_gptxt.text_identifier, parser.last_gptxt.message);
                Serial.println(c_buffer);
                break;
            case NMEAParser::UNKNOWN:
            case NMEAParser::TYPE_PLSR2451:
            case NMEAParser::TYPE_PLSR2452:
            case NMEAParser::TYPE_PLSR2457:
            case NMEAParser::TYPE_GPGGA:
            case NMEAParser::TYPE_GNGGA:
            case NMEAParser::TYPE_GPGSA:
            case NMEAParser::TYPE_GNGSA:
            case NMEAParser::TYPE_HCHDG:
            case NMEAParser::TYPE_GPGLL:
            case NMEAParser::TYPE_GNGLL:
            case NMEAParser::TYPE_GPVTG:
            case NMEAParser::TYPE_GNVTG:
            //empty/to be implemented
            case NMEAParser::TYPE_GNZDA:
            case NMEAParser::TYPE_GLGSV:
            case NMEAParser::TYPE_BDGSV:            
                break;
            }
        }
        else
        {
            Serial.printf("\n Couldn't parse, char0: %c, string:  \"", *s.c_str());
            Serial.print(s);
            Serial.println("\"\n");
        }
    }

    //display_info();
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#ifdef LILYGO_WATCH_2020_V2
static void display_info(void)
{
    static uint32_t updatePeriod;
    if (millis() - updatePeriod < 1000)
    {
        return;
    }
    updatePeriod = millis();

    if (gps->date.isUpdated() && gps->time.isUpdated())
    {
        tft->fillRect(0, 115, 240, 35, TFT_BLACK);
        tft->setCursor(0, 116);
        tft->print("DATE Year:");
        tft->print(gps->date.year());
        tft->print(" Month:");
        tft->print(gps->date.month());
        tft->print(" Day:");
        tft->println(gps->date.day());

        tft->print(F("TIME Hour:"));
        tft->print(gps->time.hour());
        tft->print(F(" Minute:"));
        tft->print(gps->time.minute());
        tft->print(F(" Second:"));
        tft->print(gps->time.second());

        // Serial.print(F("DATE       Fix Age="));
        // Serial.print(gps->date.age());
        // Serial.print(F("ms Raw="));
        // Serial.print(gps->date.value());
        // Serial.print(F(" Year="));
        // Serial.print(gps->date.year());
        // Serial.print(F(" Month="));
        // Serial.print(gps->date.month());
        // Serial.print(F(" Day="));
        // Serial.println(gps->date.day());

        // Serial.print(F("TIME       Fix Age="));
        // Serial.print(gps->time.age());
        // Serial.print(F("ms Raw="));
        // Serial.print(gps->time.value());
        // Serial.print(F(" Hour="));
        // Serial.print(gps->time.hour());
        // Serial.print(F(" Minute="));
        // Serial.print(gps->time.minute());
        // Serial.print(F(" Second="));
        // Serial.print(gps->time.second());
        // Serial.print(F(" Hundredths="));
        // Serial.println(gps->time.centisecond());
    }
    if (gps->location.isUpdated())
    {
        tft->fillRect(0, 150, 240, 20, TFT_BLACK);
        tft->setCursor(0, 150);
        tft->print(F("LOCATION  Lat:"));
        tft->print(gps->location.lat(), 3);
        tft->print(F(" Long="));
        tft->println(gps->location.lng(), 3);

        // Serial.print(F("LOCATION   Fix Age="));
        // Serial.print(gps->location.age());
        // Serial.print(F("ms Raw Lat="));
        // Serial.print(gps->location.rawLat().negative ? "-" : "+");
        // Serial.print(gps->location.rawLat().deg);
        // Serial.print("[+");
        // Serial.print(gps->location.rawLat().billionths);
        // Serial.print(F(" billionths],  Raw Long="));
        // Serial.print(gps->location.rawLng().negative ? "-" : "+");
        // Serial.print(gps->location.rawLng().deg);
        // Serial.print("[+");
        // Serial.print(gps->location.rawLng().billionths);
        // Serial.print(F(" billionths],  Lat="));
        // Serial.print(gps->location.lat(), 6);
        // Serial.print(F(" Long="));
        // Serial.println(gps->location.lng(), 6);
    }
    if (gps->speed.isUpdated())
    {

        // Serial.print(F("SPEED      Fix Age="));
        // Serial.print(gps->speed.age());
        // Serial.print(F("ms Raw="));
        // Serial.print(gps->speed.value());
        // Serial.print(F(" Knots="));
        // Serial.print(gps->speed.knots());
        // Serial.print(F(" MPH="));
        // Serial.print(gps->speed.mph());
        // Serial.print(F(" m/s="));
        // Serial.print(gps->speed.mps());
        // Serial.print(F(" km/h="));
        // Serial.println(gps->speed.kmph());
    }
    if (gps->course.isUpdated())
    {

        // Serial.print(F("COURSE     Fix Age="));
        // Serial.print(gps->course.age());
        // Serial.print(F("ms Raw="));
        // Serial.print(gps->course.value());
        // Serial.print(F(" Deg="));
        // Serial.println(gps->course.deg());
    }
    if (gps->altitude.isUpdated())
    {

        // Serial.print(F("ALTITUDE   Fix Age="));
        // Serial.print(gps->altitude.age());
        // Serial.print(F("ms Raw="));
        // Serial.print(gps->altitude.value());
        // Serial.print(F(" Meters="));
        // Serial.print(gps->altitude.meters());
        // Serial.print(F(" Miles="));
        // Serial.print(gps->altitude.miles());
        // Serial.print(F(" KM="));
        // Serial.print(gps->altitude.kilometers());
        // Serial.print(F(" Feet="));
        // Serial.println(gps->altitude.feet());
    }
    if (gps->satellites.isUpdated())
    {

        // Serial.print(F("SATELLITES Fix Age="));
        // Serial.print(gps->satellites.age());
        // Serial.print(F("ms Value="));
        // Serial.println(gps->satellites.value());
    }
    if (gps->hdop.isUpdated())
    {

        // Serial.print(F("HDOP       Fix Age="));
        // Serial.print(gps->hdop.age());
        // Serial.print(F("ms Value="));
        // Serial.println(gps->hdop.value());
    }
    delay(20);
}
#endif

static void devScan(TwoWire &w)
{
    uint8_t err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++)
    {
        w.beginTransmission(addr);
        err = w.endTransmission();
        if (err == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            switch (addr)
            {
            case MAX30105_ADDRESS:
                find_max = true;
                break;
            case PCF8563_ADDRESS:
                find_pcf = true;
                break;
            case BMA423_ADDRESS:
                find_bma = true;
                break;
            case AXP202_ADDRESS:
                find_pwr = true;
                break;
            case FT6206_ADDRESS:
                find_tp = true;
                break;
            case MPU6050_ADDRESS:
                find_mpu = true;
                break;
            case DRV2605_ADDRESS:
                find_drv = true;
            default:
                break;
            }
            nDevices++;
        }
        else if (err == 4)
        {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
}

static void resultOutput(String str, bool ret)
{
    tft->setTextColor(TFT_WHITE);
    tft->drawString(str, 0, tft->getCursorY());
    tft->setTextColor(ret ? TFT_GREEN : TFT_RED);
    tft->drawRightString(ret ? "[OK]" : "[FAIL]", tft->width() - 5, tft->getCursorY(), tft->textfont);
    tft->println();
}
