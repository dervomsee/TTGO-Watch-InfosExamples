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
static void devScan(TwoWire &w);

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

    ttgo->trunOnGPS();
    ttgo->gps_begin();
    gps = ttgo->gps;

    tft->fillScreen(TFT_BLACK);
    tft->setTextColor(TFT_WHITE);
    tft->setCursor(0, 0);
    tft->drawString("GPS sats: ", 0, 20);
    tft->drawString("GLONASS sats: ", 0, 40);
    tft->drawString("BAIDOU sats: ", 0, 60);
    tft->drawString("GNRMC: ", 0, 80);
    tft->setTextColor(TFT_GREEN);
    tft->drawString(String(0), 80, 20);
    tft->drawString(String(0), 80, 40);
    tft->drawString(String(0), 80, 60);
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
            case NMEAParser::TYPE_GNRMC:
                //show some of the sentence's data
                Serial.printf("GNRMC %s %c / %s %c\n", parser.last_gnrmc.latitude, parser.last_gnrmc.north_south_indicator, parser.last_gnrmc.longitude, parser.last_gnrmc.east_west_indicator);
                tft->fillRect(80, 80, 150, 30, TFT_BLACK);
                tft->drawString( String(parser.last_gnrmc.latitude) + " (ddmm.mmmm) " + 
                                 String(parser.last_gnrmc.north_south_indicator), 80, 80);
                tft->drawString( String(parser.last_gnrmc.longitude) + " (ddmm.mmmm) " + 
                                 String(parser.last_gnrmc.east_west_indicator), 80, 100);
                break;
            case NMEAParser::TYPE_GPGSV:
                //show some of the sentence's data
                //GPGSV sentences most of the time comes in pair, sometimes more, as each of these can give info about 4 sats at a time.
                //So let's display this header only once
                if (parser.last_gpgsv.message_idx == 1)
                {
                    Serial.printf("Found GPGSV sentence. GPS can see %d satellites\n", parser.last_gpgsv.sats_in_view);
                    tft->fillRect(80, 20, 10, 10, TFT_BLACK);
                    tft->drawString(String(parser.last_gpgsv.sats_in_view), 80, 20);
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
            case NMEAParser::TYPE_GLGSV:
                //show some of the sentence's data
                //GPGSV sentences most of the time comes in pair, sometimes more, as each of these can give info about 4 sats at a time.
                //So let's display this header only once
                if (parser.last_gpgsv.message_idx == 1)
                {
                    Serial.printf("Found GLGSV sentence. GLONASS can see %d satellites\n", parser.last_gpgsv.sats_in_view);
                    tft->fillRect(80, 40, 10, 10, TFT_BLACK);
                    tft->drawString(String(parser.last_gpgsv.sats_in_view), 80, 40);
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
            case NMEAParser::TYPE_BDGSV:
                //show some of the sentence's data
                //BDGSV sentences most of the time comes in pair, sometimes more, as each of these can give info about 4 sats at a time.
                //So let's display this header only once
                if (parser.last_bdgsv.message_idx == 1)
                {
                    Serial.printf("Found BDGSV sentence. BAIDOU can see %d satellites\n", parser.last_bdgsv.sats_in_view);
                    tft->fillRect(80, 60, 10, 10, TFT_BLACK);
                    tft->drawString(String(parser.last_bdgsv.sats_in_view), 80, 60);
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
                if(strcmp(parser.last_gptxt.message, "ANTENNA OPEN") != 0)
                    Serial.printf("TYPE_GPTXT number_of_messages: %d, sentence_number %d, text_identifier: %d, message: %s\n", parser.last_gptxt.number_of_messages, parser.last_gptxt.sentence_number, parser.last_gptxt.text_identifier, parser.last_gptxt.message);
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
            case NMEAParser::TYPE_GPRMC:
            //empty/to be implemented
            case NMEAParser::TYPE_GNZDA:           
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

}

/**********************
 *   STATIC FUNCTIONS
 **********************/
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
