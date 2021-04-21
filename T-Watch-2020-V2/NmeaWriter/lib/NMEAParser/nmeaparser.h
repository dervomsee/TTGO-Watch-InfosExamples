#pragma once

#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include <stdint.h>
	#include <string>
#endif
#include <stdarg.h>

/*
    Arduino types
    char = 1 byte
    byte = 1 byte, unsigned by default
    int16_t = 2bytes, signed by default
    word = unsigned int
    long = 4 bytes, signed
*/

typedef unsigned char byte;

/*
    Checksum = 8bit exclusive OR of all characters, excluding '$' and '*'
*/

/*
    $PLSR,245,1
    PLSR Compass measurement report 1: calibration and acceleration.
    Rate: default 1Hz, maximum 5Hz
    Sentence ID         $PLSR,245,1
    Direction           210 (degree. Magnetic direction, 0-360, 0=north)
    Calibration status  0 (auto calibration status)
    Field intensity     1090 (0..1300, magnetic field intensity, 0microTesla to 520microTesla)
    Acceleration X      22 (-512 to 511 -> -2G to 2G)
    Acceletation Y      -15 (-512 to 511 -> -2G to 2G)
    Acceleration Z      263 (-512 to 511 -> -2G to 2G)
    Temperature         19 (Celsius)
    Mounting Mode       0
    Current Calibration 2 (non zero: valid, 0=not valid)
    Checksum            *1D
*/
struct PLSR2451 {
    bool isValid;           //this tells is struct's content is issued from a valid sentence. If not, do not use struct's datas.
    int16_t fieldValidity;      //each bit of this int16_t telle if the corresponding field is valid of not.

    int16_t direction;
    int16_t calibration_status;
    int16_t field_intensity;
    int16_t acceleration_x;
    int16_t acceleration_y;
    int16_t acceleration_z;
    int16_t temperature;
    int16_t mounting_mode;
    int16_t current_calibration;
};

/*
    $PLSR,245,2
    PLSR Compass measurement report 2: attitude. The PLSR compass measurement report 2 contains
    a set of the attitude vectors, each row of the matrix means attitude vectors and it is
    normalized with 0x1000.
    Rate: 1hz, maximum 5Hz
    Sentence ID         $PLSR,245,2,
    Xx                  -3520 (X acceleration on X axis)
    Yx                  2066 (Y acceleration on X axis)
    Zx                  340 (...)
    Xy                  -2087
    Yy                  -3516
    Zy                  -232
    Xz                  174
    Yz                  -372
    Zz                  4075
    Checksum            *11
*/
struct PLSR2452 {
    bool isValid;
    int16_t fieldValidity;

    int16_t xx;
    int16_t yx;
    int16_t zx;
    int16_t xy;
    int16_t yy;
    int16_t zy;
    int16_t xz;
    int16_t yz;
    int16_t zz;
};

/*
    $PLSR,245,7
    3D GPS Speed output (ECEF coordinate)
    Sentence ID         $PLSR,245,7,
    GPS Speed (east)    11
    GPS Speed (north)   -6
    GPS Speed (up)      9
    Checksum             *17
*/
struct PLSR2457 {
    bool isValid;
    int16_t fieldValidity;

    int16_t gps_speed_east;
    int16_t gps_speed_north;
    int16_t gps_speed_up;
};

/*
    $GPGGA
    Global positioning system fixed data
    Message ID          $GPGGA
    UTC Time            171102.081 (hhmmss.sss)
    Latitude            4835.7692 (ddmm.mmmm)
    N/S indicator       N (N=north, S=south)
    Longitude           00220.9694 (dddmm.mmmm)
    E/W indocator       E (E=east, W=west)
    Position fix        1  (0 = invalid
                            1 = GPS fix (SPS)
                            2 = DGPS fix
                            3 = PPS fix
                            4 = Real Time Kinematic
                            5 = Float RTK
                            6 = estimated (dead reckoning)
                            7 = Manual input mode
                            8 = Simulation mode)
    Satellites used     04 (0 to 12)
    HDOP                3.7 (horizontal dilution of precision)
    MSL Altitude        101.6 (Above Mean Sea Level)
    Unit                M (meters)
    Geoid separation    47.4 (Height of geoid (mean sea level) above WGS84 ellipsoid
    Unit                M (meters)
    Age of diff corr.   (tims since the last DGPS update)
    Station ID          0000 (DGPS Station ID number)
    Checksum            *52
*/
struct GPGGA {
    bool isValid;
    int16_t fieldValidity;

    char utc_time[15];
    char latitude[15];
    char north_south_indicator;
    char longitude[15];
    char east_west_indicator;
    int16_t position_fix;
    int16_t satellites_used;
    float hdop;
    float msl_altitude;
    char msl_unit[5];
    float geoid_separation;
    char geoid_unit[5];
    float age_of_diff_corr;
    int16_t station_id;
};
struct GNGGA {
    bool isValid;
    int16_t fieldValidity;

    char utc_time[15];
    char latitude[15];
    char north_south_indicator;
    char longitude[15];
    char east_west_indicator;
    int16_t position_fix;
    int16_t satellites_used;
    float hdop;
    float msl_altitude;
    char msl_unit[5];
    float geoid_separation;
    char geoid_unit[5];
    float age_of_diff_corr;
    int16_t station_id;
};

/*
    $GPGSA
    GNSS DOP and active satellites
    Sentence ID         $GPGSA
    Mode 1              A (A=automatic, allowed to switch 2D/3D, M=manual, forced to operate in 2D/3D)
    Mode 2              3 (1=fix not available, 2=2D, 3=3D)
    Id of satellite     19 (Satellite used on channel 1)
    Id of satellite     03 (Satellite used on channel 2)
    Id of satellite     14 (Satellite used on channel 3)
    Id of satellite     32 (Satellite used on channel 4)
    Id of satellite      (Satellite used on channel 5)
    Id of satellite      (Satellite used on channel 6)
    Id of satellite      (Satellite used on channel 7)
    Id of satellite      (Satellite used on channel 8)
    Id of satellite      (Satellite used on channel 9)
    Id of satellite      (Satellite used on channel 10)
    Id of satellite      (Satellite used on channel 11)
    Id of satellite      (Satellite used on channel 12)
    PDOP                5.7 (Position Dilution of Precision, 1.0 is perfect)
    HDOP                3.7 (Horizontal Dilution of Precision, 1.0 is perfect)
    VDOP                4.3 (Vertical Dilution of Precision, 1.0 is perfect)
    Checksum            *3C
*/
struct GPGSA {
    bool isValid;
    int16_t fieldValidity;

    char mode1;
    int16_t mode2;
    int16_t sat_channel_1;
    int16_t sat_channel_2;
    int16_t sat_channel_3;
    int16_t sat_channel_4;
    int16_t sat_channel_5;
    int16_t sat_channel_6;
    int16_t sat_channel_7;
    int16_t sat_channel_8;
    int16_t sat_channel_9;
    int16_t sat_channel_10;
    int16_t sat_channel_11;
    int16_t sat_channel_12;
    float pdop;
    float hdop;
    float vdop;
};
struct GNGSA {
    bool isValid;
    int16_t fieldValidity;

    char mode1;
    int16_t mode2;
    int16_t sat_channel_1;
    int16_t sat_channel_2;
    int16_t sat_channel_3;
    int16_t sat_channel_4;
    int16_t sat_channel_5;
    int16_t sat_channel_6;
    int16_t sat_channel_7;
    int16_t sat_channel_8;
    int16_t sat_channel_9;
    int16_t sat_channel_10;
    int16_t sat_channel_11;
    int16_t sat_channel_12;
    float pdop;
    float hdop;
    float vdop;
};

/*
    $GPGSV
    Satellites in View. SNR (Signal to Noise Ratio) field is often used as "Signal Strength", higher is better.
    Multiple sentences can be emitted, 4 satellites per sentence.
    Sentence ID         $GPGSV
    Number of GSV msgs  3 (from 1 to 3)
    Message number      1 (from 1 to 3)
    Satellites in view  12
    Satellite ID        11 (channel 1, from 01 to 32)
    Elevation           77 ((degrees) channel 1, from 01 to 90)
    Azimuth             288 ((degrees) channel 1, from 000 to 359)
    SNR                 ((db-Hz) channel 1, from 00 to 99, null when not tracking)
    Satellite ID        32 (channel 2, from 01 to 32)
    Elevation           58 ((degrees) channel 2, from 01 to 90)
    Azimuth             207 ((degrees) channel 2, from 000 to 359)
    SNR                 34 ((db-Hz) channel 2, from 00 to 99, null when not tracking)
    Satellite ID        19 (channel 3, from 01 to 32)
    Elevation           49 ((degrees) channel 3, from 01 to 90)
    Azimuth             154 ((degrees) channel 3, from 000 to 359)
    SNR                 40 ((db-Hz) channel 3, from 00 to 99, null when not tracking)
    Satellite ID        14 (channel 4, from 01 to 32)
    Elevation           35 ((degrees) channel 4, from 01 to 90)
    Azimuth             074 ((degrees) channel 4, from 000 to 359)
    SNR                 39 ((db-Hz) channel 4, from 00 to 99, null when not tracking)
    Checksum            *7F
*/
struct GPGSV {
    bool isValid;
    int16_t fieldValidity;

    int16_t number_of_messages;
    int16_t message_idx;
    int16_t sats_in_view;
    int16_t sat1_id;
    int16_t sat1_elevation;
    int16_t sat1_azimuth;
    int16_t sat1_snr;
    int16_t sat2_id;
    int16_t sat2_elevation;
    int16_t sat2_azimuth;
    int16_t sat2_snr;
    int16_t sat3_id;
    int16_t sat3_elevation;
    int16_t sat3_azimuth;
    int16_t sat3_snr;
    int16_t sat4_id;
    int16_t sat4_elevation;
    int16_t sat4_azimuth;
    int16_t sat4_snr;
};
struct GLGSV {
    bool isValid;
    int16_t fieldValidity;

    int16_t number_of_messages;
    int16_t message_idx;
    int16_t sats_in_view;
    int16_t sat1_id;
    int16_t sat1_elevation;
    int16_t sat1_azimuth;
    int16_t sat1_snr;
    int16_t sat2_id;
    int16_t sat2_elevation;
    int16_t sat2_azimuth;
    int16_t sat2_snr;
    int16_t sat3_id;
    int16_t sat3_elevation;
    int16_t sat3_azimuth;
    int16_t sat3_snr;
    int16_t sat4_id;
    int16_t sat4_elevation;
    int16_t sat4_azimuth;
    int16_t sat4_snr;
};
struct BDGSV {
    bool isValid;
    int16_t fieldValidity;

    int16_t number_of_messages;
    int16_t message_idx;
    int16_t sats_in_view;
    int16_t sat1_id;
    int16_t sat1_elevation;
    int16_t sat1_azimuth;
    int16_t sat1_snr;
    int16_t sat2_id;
    int16_t sat2_elevation;
    int16_t sat2_azimuth;
    int16_t sat2_snr;
    int16_t sat3_id;
    int16_t sat3_elevation;
    int16_t sat3_azimuth;
    int16_t sat3_snr;
    int16_t sat4_id;
    int16_t sat4_elevation;
    int16_t sat4_azimuth;
    int16_t sat4_snr;
};

/*
    $HCHDG
    Deviation and variation.
    Rate: default 1Hz, maxmimum 1Hz.
    Sentence ID         $HCHDG
    Heading             210 ((degree) magnetic sensor heading)
    Deviation           ((degree) magnetic deviation)
    Deviation direction (E=easterly, W=westerly)
    Variation           0.2 ((degree) magnetic variation)
    Variation direction W (E=easterly, W=westerly)
    Checksum            *24
*/
struct HCHDG {
    bool isValid;
    int16_t fieldValidity;

    int16_t heading;
    float deviation;
    char dev_direction;
    float variation;
    char var_direction;
};

/*
    $GPRMC
    Recommended Minimum Specific GNSS Data
    Sentence ID         $GPRMC
    UTC Time            171102.081 (hhmmss.sss)
    Status              A (A=datavalid, V=data not valid)
    Latitude            4835.7692 (ddmm.mmmm)
    N/S indicator       N (N=north, S=south)
    Longitude           00220.9694 (dddmm.mmmm)
    E/W Indicator       E (E=east, W=West)
    Speed over ground   0.24 (knots)
    Course over ground  210 ((degrees) Not valid at low speed. LS20126 may derive it from magnetic heading in this case)
    Date                121011 (ddmmyy)
    Magnetic variation  0.2 (degree)
    Variation sense     W (W=west, E=east)
    Mode                A (A=autonomous, D=DGPS, E=DR)
    Checksum            *33
*/
struct GPRMC {
    bool isValid;
    int16_t fieldValidity;

    char utc_time[15];
    char status;
    char latitude[15];
    char north_south_indicator;
    char longitude[15];
    char east_west_indicator;
    float speed_over_ground;
    float course_over_ground;
    char date[10];
    float magnetic_variation;
    char variation_sense;
    char mode;
};

/*
    $GNRMC
    Recommended Minimum Specific GNSS Data
    Sentence ID         $GNRMC
    UTC Time            171102.081 (hhmmss.sss)
    Status              A (A=datavalid, V=data not valid)
    Latitude            4835.7692 (ddmm.mmmm)
    N/S indicator       N (N=north, S=south)
    Longitude           00220.9694 (dddmm.mmmm)
    E/W Indicator       E (E=east, W=West)
    Speed over ground   0.24 (knots)
    Course over ground  210 ((degrees) Not valid at low speed. LS20126 may derive it from magnetic heading in this case)
    Date                121011 (ddmmyy)
    Magnetic variation  0.2 (degree)
    Variation sense     W (W=west, E=east)
    Mode                A (A=autonomous, D=DGPS, E=DR)
    Checksum            *33
*/
struct GNRMC {
    bool isValid;
    int16_t fieldValidity;

    char utc_time[15];
    char status;
    char latitude[15];
    char north_south_indicator;
    char longitude[15];
    char east_west_indicator;
    float speed_over_ground;
    float course_over_ground;
    char date[10];
    float magnetic_variation;
    char variation_sense;
    char mode;
};

/*
    $GPGLL
    Geographic Position, Latitude / Longitude and time.
    $GPGLL,4448.55381,N,00038.83314,W,134006.00,A,A*7B
    Sentence ID         $GPGLL
    Latitude            4448.55381 (ddmm.mmmmm)
    N/S indicator       N (N=north, S=south)
    Longitude           00048.83314 (dddmm.mmmm)
    E/W Indicator       W (E=east, W=West)
    Fix time            134006.00 (13h40mn06.00s UTC)
    Data Active or void A (or V)
*/
struct GPGLL {
    bool isValid;
    int16_t fieldValidity;

    char latitude[15];
    char north_south_indicator;
    char longitude[15];
    char east_west_indicator;
    char fix_time[15];
    char data_active;
};
struct GNGLL {
    bool isValid;
    int16_t fieldValidity;

    char latitude[15];
    char north_south_indicator;
    char longitude[15];
    char east_west_indicator;
    char fix_time[15];
    char data_active;
};

/*
    $GPVTG
    VTG - Velocity made good.
    $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
    *48          Checksum

    An extra field might exit right before checksum:
    Mode: A=Autonomous, D=DGPS, E=DR
*/
struct GPVTG {
    bool isValid;
    int16_t fieldValidity;

    float measured_heading_1;
    char north_type_1;
    float measured_heading_2;
    char north_type_2;
    float ground_speed_1;
    char ground_speed_unit_1;
    float ground_speed_2;
    char ground_speed_unit_2;

    char mode;
};
struct GNVTG {
    bool isValid;
    int16_t fieldValidity;

    float measured_heading_1;
    char north_type_1;
    float measured_heading_2;
    char north_type_2;
    float ground_speed_1;
    char ground_speed_unit_1;
    float ground_speed_2;
    char ground_speed_unit_2;

    char mode;
};

/*
    $GPTXT
    Text message
    $GPTXT,01,01,02,u-blox ag - www.u-blox.com*50
    01                          Total number of messages, 01 to 99
    01                          Sentence number, 01 to 99
    02                          Text identifier, 01 to 99
    u-blox ag - www.u-blox.com  Text message, up to 61 characters
    *50                         Checksum
*/
struct GPTXT {
    bool isValid;
    int16_t fieldValidity;

    uint8_t number_of_messages;
    uint8_t sentence_number;
    uint8_t text_identifier;
    char message[70];
};

#define GET_BIT(x, pos) (((x)&(1<<(pos)))!=0)
#define SET_BIT(x, pos) ((x)|(1<<(pos)))
#define IS_DIGIT(c) ((c)>='0' && (c)<='9')

class NMEAParser {
public:
    NMEAParser();

    bool check_checksum(const char *str);
    char generate_checksum(const char *str);

    //central dispatcher
#ifdef ARDUINO
	bool dispatch(const String str);
#else
	bool dispatch(const std::string str);
#endif
    bool dispatch(const char *str);

    //types
    enum STRINGS_TYPES {
        UNKNOWN=0,
        TYPE_PLSR2451,
        TYPE_PLSR2452,
        TYPE_PLSR2457,
        TYPE_GPGGA,
        TYPE_GNGGA,
        TYPE_GPGSA,
        TYPE_GNGSA,
        TYPE_GPGSV,
        TYPE_HCHDG,
        TYPE_GPRMC,
        TYPE_GNRMC,
        TYPE_GPGLL,
        TYPE_GNGLL,
        TYPE_GPVTG,
        TYPE_GNVTG,
        TYPE_GPTXT,
        //new/empty
        TYPE_GNZDA,
        TYPE_GLGSV,
        TYPE_BDGSV,        
    };
    STRINGS_TYPES getLastProcessedType() {return last_processed;}


    //parsers
    bool parse_plsr2451(const char *str);
    bool parse_plsr2452(const char *str);
    bool parse_plsr2457(const char *str);
    bool parse_gpgga(const char *str);
    bool parse_gngga(const char *str);
    bool parse_gpgsa(const char *str);
    bool parse_gngsa(const char *str);
    bool parse_gpgsv(const char *str);
    bool parse_glgsv(const char *str);
    bool parse_bdgsv(const char *str);
    bool parse_hchdg(const char *str);
    bool parse_gprmc(const char *str);
    bool parse_gnrmc(const char *str);
    bool parse_gpgll(const char *str);
    bool parse_gngll(const char *str);
    bool parse_gpvtg(const char *str);
    bool parse_gnvtg(const char *str);
    bool parse_gptxt(const char *str);

    PLSR2451 last_plsr2451 = {};
    PLSR2452 last_plsr2452 = {};
    PLSR2457 last_plsr2457 = {};
    GPGGA    last_gpgga = {};
    GNGGA    last_gngga = {};
    GPGSA    last_gpgsa = {};
    GNGSA    last_gngsa = {};
    GPGSV    last_gpgsv = {};
    GLGSV    last_glgsv = {};
    BDGSV    last_bdgsv = {};
    HCHDG    last_hchdg = {};
    GPRMC    last_gprmc = {};
    GNRMC    last_gnrmc = {};
    GPGLL    last_gpgll = {};
    GNGLL    last_gngll = {};
    GPVTG    last_gpvtg = {};
    GNVTG    last_gnvtg = {};
    GPTXT    last_gptxt = {};

private:
    STRINGS_TYPES last_processed = UNKNOWN;

    int16_t checksum = 0;
    int16_t scanned = 0;

    //string parsing subfunctions
    int16_t my_atoh(char a);
    char *my_strncpy(char *dst, const char *src, int16_t n);
    int16_t my_strlen(const char* str);
    int32_t my_atoi(const char *str);
    float my_atof(const char *s);

    //main parser
    int16_t my_sscanf(int16_t *field_validity, const char *src, const char *format, ... );

    //calculus tools
};
