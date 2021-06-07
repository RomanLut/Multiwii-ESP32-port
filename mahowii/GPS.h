#ifndef GPS_H_
#define GPS_H_

// Based on average Earth radius: lat_lon_to_cm = pi/180 * Earth_radius / 10^7, where volumetric mean radius 6371 km
#define LAT_LON_TO_CM     1.11318845f

//extern int32_t gpsPositionError[2];
extern int32_t gpsDistanceToHome[2];
extern int16_t gpsActualSpeed[2];

//Function prototypes for GPS frame parsing
bool GPS_newFrame(uint8_t c);
extern uint8_t GPS_Frame;            // a valid GPS_Frame was detected, and data is ready for nav computation

extern int32_t wrap_18000(int32_t ang);

void GPS_set_pids(void);
void GPS_SerialInit(void);
bool GPS_Compute(void);
void GPS_reset_home_position(void);
void GPS_set_next_wp(int32_t* lat_to, int32_t* lon_to, int32_t* lat_from, int32_t* lon_from);
void GPS_reset_nav(void);

bool isNavStateForPosHold();
void applyPosHoldPIDControl(float* dt);
void predictAndSetPositionToHold();

void check_altitude();
void abort_mission(unsigned char error_code);
void GPS_adjust_heading();
void init_RTH(void);


#if defined(I2C_GPS)
uint8_t GPS_NewData(void);
#endif

extern uint32_t wp_distance;
extern int32_t target_bearing;
#endif /* GPS_H_ */
