#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "GPS.h"
#include "Serial.h"
#include "Sensors.h"
#include "MahoWii.h"
#include "EEPROM.h"
#include "AltHold.h"
#include "Math.h"
#include <math.h>

#if GPS

//Function prototypes for other GPS functions
//These perhaps could go to the gps.h file, however these are local to the gps.cpp
static void GPS_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* bearing);
static void GPS_distance_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, uint32_t* dist);
static void calculateDistanceToHome(uint32_t* dist);
static void GPS_calc_velocity(void);

#ifndef INS_PH_NAV_ON
	static void GPS_calc_location_error(int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng);
#endif

static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow);
static void GPS_calc_nav_rate(uint16_t max_speed);
int32_t wrap_18000(int32_t ang);
static bool check_missed_wp(void);
void GPS_calc_longitude_scaling(int32_t lat);
static void GPS_update_crosstrack(void);
int32_t wrap_36000(int32_t ang);


typedef struct PID_PARAM_ {
    float kP;
    float kI;
    float kD;
    float Imax;
} PID_PARAM;

PID_PARAM posholdPID_PARAM;
PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;

typedef struct PID_ {
    float integrator; // integrator value
    int32_t last_input; // last input for derivative
    float lastderivative; // last derivative for low-pass filter
    //float output;
    float derivative;
} PID;
PID posholdPID[2];
PID poshold_ratePID[2];
PID navPID[2];

int32_t get_P(int32_t error, struct PID_PARAM_* pid) {
    return (float) error * pid->kP;
}

int32_t get_I(int32_t error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) {
    pid->integrator += ((float) error * pid_param->kI) * *dt;
    pid->integrator = constrain(pid->integrator, -pid_param->Imax, pid_param->Imax);
    return pid->integrator;
}

int32_t get_D(int32_t input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) { // dt in milliseconds
    pid->derivative = (input - pid->last_input) / *dt;

    /// Low pass filter cut frequency for derivative calculation.
    float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
    // Examples for _filter:
    // f_cut = 10 Hz -> _filter = 15.9155e-3
    // f_cut = 15 Hz -> _filter = 10.6103e-3
    // f_cut = 20 Hz -> _filter =  7.9577e-3
    // f_cut = 25 Hz -> _filter =  6.3662e-3
    // f_cut = 30 Hz -> _filter =  5.3052e-3

    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    pid->derivative = pid->lastderivative + (*dt / (filter + *dt)) * (pid->derivative - pid->lastderivative);
    // update state
    pid->last_input = input;
    pid->lastderivative = pid->derivative;
    // add in derivative component
    return pid_param->kD * pid->derivative;
}

void reset_PID(struct PID_* pid) {
    pid->integrator = 0;
    pid->last_input = 0;
    pid->lastderivative = 0;
}

#define _X 1
#define _Y 0

//#define RADX100                    0.000174532925


uint8_t GPS_Frame;            // a valid GPS_Frame was detected, and data is ready for nav computation

static float dTnav;            // Delta Time in milliseconds for navigation computations, updated with every good GPS read
int16_t gpsActualSpeed[2] = { 0, 0 };
static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t rate_error[2];
int32_t gpsDistanceToHome[2];
#ifdef INS_PH_NAV_ON
int32_t positionToHold[2];
#else
int32_t gpsPositionError[2];
#endif

static int32_t GPS_WP[2];   //Currently used WP
static int32_t GPS_FROM[2]; //the pervious waypoint for precise track following
int32_t target_bearing;     // This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t original_target_bearing;  // deg * 100, The original angle to the next_WP when the next_WP was set, Also used to check when we pass a WP
static int16_t crosstrack_error;     // The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
uint32_t wp_distance;                // distance between plane and next_WP in cm
static uint16_t waypoint_speed_gov;  // used for slow speed wind up when start navigation;

////////////////////////////////////////////////////////////////////////////////////
// moving average filter variables
////////////////////////////////////////////////////////////////////////////////////

#if defined(GPS_FILTERING) && (!defined(INS_PH_NAV_ON))

	#define GPS_FILTER_VECTOR_LENGTH 5

	static uint8_t GPS_filter_index = 0;
	static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
	static int32_t GPS_filter_sum[2];
	static int32_t GPS_read[2];
	static int32_t GPS_filtered[2];
	static int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
	static uint16_t fraction3[2];

#endif

static int16_t nav_takeoff_bearing;  // saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home

//Main navigation processor and state engine
// TODO: add proceesing states to ease processing burden
bool GPS_Compute(void) {
    unsigned char axis;
    uint32_t dist;        //temp variable to store dist to copter
    int32_t dir;         //temp variable to store dir to copter

    //check that we have a valid frame, if not then return immediatly
    if (GPS_Frame == 0) {
        return false;
    } else {
        GPS_Frame = 0;
    }

    if (f.GPS_FIX && GPS_numSat >= 4) {

    	//check home position and set it if it was not set
#if !defined(DONT_RESET_HOME_AT_ARM)
        if (!f.ARMED) {
            f.GPS_FIX_HOME = 0;
        }
#endif
        if (!f.GPS_FIX_HOME && f.ARMED) {
            GPS_reset_home_position();
        }

#if defined(GPS_FILTERING) && (!defined(INS_PH_NAV_ON))
        //Apply moving average filter to GPS data
        //if (GPS_conf.filtering) {
            GPS_filter_index = (GPS_filter_index + 1) % GPS_FILTER_VECTOR_LENGTH;
            for (axis = 0; axis < 2; axis++) {
                GPS_read[axis] = GPS_coord[axis]; //latest unfiltered data is in GPS_latitude and GPS_longitude
                GPS_degree[axis] = GPS_read[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t

                // How close we are to a degree line ? its the first three digits from the fractions of degree
                // later we use it to Check if we are close to a degree line, if yes, disable averaging,
                fraction3[axis] = (GPS_read[axis] - GPS_degree[axis] * 10000000) / 10000;

                GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
                GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis] * 10000000);
                GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
                GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis] * 10000000);
                if (NAV_state == NAV_STATE_HOLD_INFINIT || NAV_state == NAV_STATE_HOLD_TIMED) {      //we use gps averaging only in poshold mode...
                    if (fraction3[axis] > 1 && fraction3[axis] < 999)
                        GPS_coord[axis] = GPS_filtered[axis];
                }
            }
        //}
#endif

        //dTnav calculation
        //Time for calculating x,y speed and navigation pids
        static uint32_t prevNavTime;
        uint32_t currTime = millis();
        dTnav = MS2S(currTime - prevNavTime);
        prevNavTime = currTime;
        //debug[1] = dTnav * 1000;

        // prevent runup from bad GPS
        dTnav = min(dTnav, 1.0);

        //calculate distance and bearings for gui and other stuff continously - From home to copter
        GPS_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dir);
        GPS_directionToHome = dir / 100;

        //GPS_distance_cm(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dist);
        calculateDistanceToHome(&dist);
        GPS_distanceToHome = dist / 100;

        if (!f.GPS_FIX_HOME) {     //If we don't have home set, do not display anything
            GPS_distanceToHome = 0;
            GPS_directionToHome = 0;
        }

        // Check fence setting and execute RTH if necessary
        if ((GPS_conf.fence > 0) && (GPS_conf.fence < GPS_distanceToHome) && (f.GPS_mode != GPS_MODE_RTH)) {
        	mission_step.parameter1 = 1; 		// make auto landing after RTH
        	init_RTH();
        }

        //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
        GPS_calc_velocity();

        //Navigation state engine
        if (f.GPS_mode != GPS_MODE_NONE) {   //ok we are navigating ###0002
            //do gps nav calculations here, these are common for nav and poshold
            GPS_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &target_bearing);
			GPS_distance_cm(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance);
			#ifndef INS_PH_NAV_ON
				GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);
			#endif

            int16_t speed = 0;                   //Desired navigation speed

        switch (NAV_state) {                    //Navigation state machine

            case NAV_STATE_NONE:               //Just for clarity, do nothing when nav_state is none
                break;

            case NAV_STATE_LAND_START:
            	f.GPS_BARO_MODE = true;          //Take control of BARO mode
            	f.LAND_COMPLETED = false;
            	setAltToHold(alt.estAlt);
			  #ifndef INS_PH_NAV_ON
            	applyPosHoldPIDControl(&dTnav);              //Land in position hold
            	nav_timer_stop = millis() + 1000; // currTime + 1s
			  #else
            	nav_timer_stop = millis() + 100; // currTime + 100ms
			  #endif
            	NAV_state = NAV_STATE_LAND_SETTLE;
                break;

            case NAV_STATE_LAND_SETTLE:
			  #ifndef INS_PH_NAV_ON
            	applyPosHoldPIDControl(&dTnav);
			  #endif
                if (millis() >= nav_timer_stop) {
                    f.LAND_IN_PROGRESS = true;            // Flag land process
                    NAV_state = NAV_STATE_LAND_IN_PROGRESS;
                }
                break;

            case NAV_STATE_LAND_IN_PROGRESS:
			  #ifndef INS_PH_NAV_ON
            	applyPosHoldPIDControl(&dTnav);              //Land in position hold
			  #endif
            	if (isLanded()) {
                	//nav_timer_stop = millis() + 2000; // currTime + 2s
                    NAV_state = NAV_STATE_LANDED;

            	} else if(isGroundDetected()) {	// if ground detected switch off position hold
            		NAV_state = NAV_STATE_LAND_DETECTED;
            	}
                break;

            case NAV_STATE_LAND_DETECTED:
				if (isLanded()) {
					//nav_timer_stop = millis() + 2000; // currTime + 2s
					NAV_state = NAV_STATE_LANDED;
				}
				break;

            case NAV_STATE_LANDED:
                // Disarm if THROTTLE stick is at minimum or 2sec past after land detected
                //if (rcData[THROTTLE] < MINCHECK || millis() >= nav_timer_stop) { // Throttle at minimum or 5sec passed.
                    go_disarm();
                    f.OK_TO_ARM = 0;                //Prevent re-arming
                    f.GPS_BARO_MODE = false;
                    f.LAND_IN_PROGRESS = false;
                    f.LAND_COMPLETED = true;
                    GPS_reset_nav();
                //}
                break;

            case NAV_STATE_HOLD_INFINIT:        //Constant position hold, no timer. Only an rcOption change can exit from this
			  #ifndef INS_PH_NAV_ON
            	applyPosHoldPIDControl(&dTnav);
			  #endif
				break;

            case NAV_STATE_HOLD_TIMED:
                if (nav_timer_stop == 0) {                         //We are start a timed poshold
                    nav_timer_stop = millis() + 1000 * nav_hold_time;  //Set when we will continue
                } else if (nav_timer_stop <= millis()) {           //did we reach our time limit ?
                    if (mission_step.flag != MISSION_FLAG_END) {
                        NAV_state = NAV_STATE_PROCESS_NEXT;            //if yes then process next mission step
                    }
                    NAV_error = NAV_ERROR_TIMEWAIT;
                }

			  #ifndef INS_PH_NAV_ON
            	applyPosHoldPIDControl(&dTnav); //BTW hold position till next command
			  #endif
                break;

            case NAV_STATE_RTH_START:
				if (GPS_distanceToHome <= RTH_RADIUS) {
					if (mission_step.parameter1 == 0) {
						setAltToHold(alt.estAlt); // just keep current alt in case of drone in RTH_RADIUS at RTH activation
						NAV_state = NAV_STATE_HOLD_INFINIT;
					} else {
						NAV_state = NAV_STATE_LAND_START; // if parameter 1 in RTH step is non 0 then land at home
					}
					if (GPS_conf.nav_rth_takeoff_heading) {
						magHold = nav_takeoff_bearing;
					}

				} else if (isAltitudeReached() || (!GPS_conf.wait_for_target_alt)) {             //Wait until we reach RTH altitude
                    GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON], &GPS_coord[LAT], &GPS_coord[LON]); //If we reached then change mode and start RTH
                    NAV_state = NAV_STATE_RTH_ENROUTE;
                    NAV_error = NAV_ERROR_NONE;

				} else {
                    NAV_error = NAV_ERROR_WAIT_FOR_TARGET_ALT;
                }

			  #ifndef INS_PH_NAV_ON
            	applyPosHoldPIDControl(&dTnav); //hold position till we reach RTH alt
			  #endif
                break;

            case NAV_STATE_RTH_ENROUTE:                                                  //Doing RTH navigation
                speed = GPS_calc_desired_speed(GPS_conf.nav_speed_max, GPS_conf.slow_nav);
                GPS_calc_nav_rate(speed);
                GPS_adjust_heading();
                if ((wp_distance <= GPS_conf.wp_radius) || check_missed_wp()) {            //if yes switch to poshold mode
                    if (mission_step.parameter1 == 0) {
                        NAV_state = NAV_STATE_HOLD_INFINIT;
                    } else {
                        NAV_state = NAV_STATE_LAND_START;                                   // if parameter 1 in RTH step is non 0 then land at home
                    }
                    if (GPS_conf.nav_rth_takeoff_heading) {
                        magHold = nav_takeoff_bearing;
                    }
                }
                break;

            case NAV_STATE_WP_START:
				if (isAltitudeReached() || (!GPS_conf.wait_for_target_alt)) { //Wait until we reach WP altitude
					GPS_set_next_wp(&mission_step.pos[LAT], &mission_step.pos[LON], &GPS_coord[LAT], &GPS_coord[LON]);
					NAV_state = NAV_STATE_WP_ENROUTE;
					NAV_error = NAV_ERROR_NONE;
				} else {
					NAV_error = NAV_ERROR_WAIT_FOR_TARGET_ALT;
				}
			  #ifndef INS_PH_NAV_ON
				applyPosHoldPIDControl(&dTnav); //hold position till we reach WP altitude
			  #endif
				break;

            case NAV_STATE_WP_ENROUTE:
                speed = GPS_calc_desired_speed(GPS_conf.nav_speed_max, GPS_conf.slow_nav);
                GPS_calc_nav_rate(speed);
                GPS_adjust_heading();

                if ((wp_distance <= GPS_conf.wp_radius) || check_missed_wp()) {               //This decides what happen when we reached the WP coordinates
                    if (mission_step.action == MISSION_LAND) {                                  //Autoland
                        NAV_state = NAV_STATE_LAND_START;                                         //Start landing
                    } else if (mission_step.flag == MISSION_FLAG_END) { //If this was the last mission step (flag set by the mission planner), then switch to poshold
                        NAV_state = NAV_STATE_HOLD_INFINIT;
                        NAV_error = NAV_ERROR_FINISH;
                    } else if (mission_step.action == MISSION_HOLD_UNLIM) { //If mission_step was POSHOLD_UNLIM and we reached the position then switch to poshold unlimited
                        NAV_state = NAV_STATE_HOLD_INFINIT;
                        NAV_error = NAV_ERROR_FINISH;
                    } else if (mission_step.action == MISSION_HOLD_TIME) {                     //If mission_step was a timed poshold then initiate timed poshold
                        nav_hold_time = mission_step.parameter1;
                        nav_timer_stop = 0;                                                       //This indicates that we are starting a timed poshold
                        NAV_state = NAV_STATE_HOLD_TIMED;
                    } else {
                        NAV_state = NAV_STATE_PROCESS_NEXT;                                       //Otherwise process next step
                    }
                }
                break;

            case NAV_STATE_DO_JUMP:
                if (jump_times < 0) {                                  //Jump unconditionally (supposed to be -1) -10 should not be here
                    next_step = mission_step.parameter1;
                    NAV_state = NAV_STATE_PROCESS_NEXT;
                }
                if (jump_times == 0) {
                    jump_times = -10;                                    //reset jump counter
                    if (mission_step.flag == MISSION_FLAG_END) {   //If this was the last mission step (flag set by the mission planner), then switch to poshold
                        NAV_state = NAV_STATE_HOLD_INFINIT;
                        NAV_error = NAV_ERROR_FINISH;
                    } else
                        NAV_state = NAV_STATE_PROCESS_NEXT;
                }

                if (jump_times > 0) {                                  //if zero not reached do a jump
                    next_step = mission_step.parameter1;
                    NAV_state = NAV_STATE_PROCESS_NEXT;
                    jump_times--;
                }
                break;

            case NAV_STATE_PROCESS_NEXT:                             //Processing next mission step
                NAV_error = NAV_ERROR_NONE;
                if (!recallWP(next_step)) {
                    abort_mission(NAV_ERROR_WP_CRC);
                } else {
                    switch (mission_step.action) {
                    //Waypoint and hold commands all starts with an enroute status it includes the LAND command too
                    case MISSION_WAYPOINT:
                    case MISSION_HOLD_TIME:
                    case MISSION_HOLD_UNLIM:
                    case MISSION_LAND:
                        setAltToHold(mission_step.altitude);
                        // all WP starts with a position hold. This allows to go to WP altitude.
                        predictAndSetPositionToHold(); // predict and set point to hold

                        if ((wp_distance / 100) >= GPS_conf.safe_wp_distance) {
                            abort_mission(NAV_ERROR_TOOFAR);
                        } else {
                            NAV_state = NAV_STATE_WP_START;
                        }
                        //GPS_prev[LAT] = mission_step.pos[LAT];  //Save wp coordinates for precise route calc
                        //GPS_prev[LON] = mission_step.pos[LON];
                        break;

                    case MISSION_RTH:
                        f.GPS_head_set = 0;
                        if (GPS_conf.rth_altitude == 0 && mission_step.altitude == 0) { //if config and mission_step alt is zero
                            setAltToHold(alt.estAlt);     // RTH returns at the actual altitude
                        } else {
                            uint32_t rth_alt;
                            if (mission_step.altitude == 0) {
                                rth_alt = GPS_conf.rth_altitude * 100;   //altitude in mission step has priority
                            } else {
                                rth_alt = mission_step.altitude;
                            }

                            if (alt.estAlt < rth_alt) {
                                setAltToHold(rth_alt);                     //BUt only if we are below it.
                            } else {
                                setAltToHold(alt.estAlt);
                            }
                        }

                        // all RTH starts with a position hold. This allows to go to RTH altitude.
                        predictAndSetPositionToHold(); // predict and set point to hold

                        NAV_state = NAV_STATE_RTH_START;
                        break;

                    case MISSION_JUMP:
                        if (jump_times == -10)
                            jump_times = mission_step.parameter2;
                        if (mission_step.parameter1 > 0 && mission_step.parameter1 < mission_step.number)
                            NAV_state = NAV_STATE_DO_JUMP;
                        else
                            //Error situation, invalid jump target
                            abort_mission(NAV_ERROR_INVALID_JUMP);
                        break;

                    case MISSION_SET_HEADING:
                        GPS_poi[LAT] = 0;
                        GPS_poi[LON] = 0; // zeroing this out clears the possible previous set_poi
                        if (mission_step.parameter1 < 0)
                            f.GPS_head_set = 0;
                        else {
                            f.GPS_head_set = 1;
                            GPS_directionToPoi = mission_step.parameter1;
                        }
                        break;

                    case MISSION_SET_POI:
                        GPS_poi[LAT] = mission_step.pos[LAT];
                        GPS_poi[LON] = mission_step.pos[LON];
                        f.GPS_head_set = 1;
                        break;

                    default:                                  //if we got an unknown action code abort mission and hold position
                        abort_mission(NAV_ERROR_INVALID_DATA);
                        break;
                    }
                    next_step++; //Prepare for the next step
                }
                break;
            } // switch end
        } //end of gps calcs ###0002
    }

    return true;
} // End of GPS_compute

// Abort current mission with the given error code (switch to poshold_infinit)
void abort_mission(unsigned char error_code) {
    GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON], &GPS_coord[LAT], &GPS_coord[LON]);
    NAV_error = error_code;
    NAV_state = NAV_STATE_HOLD_INFINIT;
}

//Adjusting heading according to settings - MAG mode must be enabled
void GPS_adjust_heading() {
    //TODO: Add slow windup for large heading change
    //This controls the heading
    if (f.GPS_head_set) { // We have seen a SET_POI or a SET_HEADING command
        if (GPS_poi[LAT] == 0)
            magHold = wrap_18000((GPS_directionToPoi * 100)) / 100;
        else {
            GPS_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_poi[LAT], &GPS_poi[LON], &GPS_directionToPoi);
            GPS_distance_cm(&GPS_coord[LAT], &GPS_coord[LON], &GPS_poi[LAT], &GPS_poi[LON], &wp_distance);
            magHold = GPS_directionToPoi / 100;
        }
    } else {                                // heading controlled by the standard defines
        if (GPS_conf.nav_controls_heading) {
            if (GPS_conf.nav_tail_first) {
                magHold = wrap_18000(target_bearing - 18000) / 100;
            } else {
                magHold = wrap_18000(target_bearing) / 100;
            }
        }
    }
}



////////////////////////////////////////////////////////////////////////////////////
//PID based GPS navigation functions
//Author : EOSBandi
//Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
//Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

//original constraint does not work with variables
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}
////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat) {
    GPS_scaleLonDown = cos(lat * 1.0e-7f * 0.01745329251f);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t* lat_to, int32_t* lon_to, int32_t* lat_from, int32_t* lon_from) {
    GPS_WP[LAT] = *lat_to;
    GPS_WP[LON] = *lon_to;

    GPS_FROM[LAT] = *lat_from;
    GPS_FROM[LON] = *lon_from;

    GPS_calc_longitude_scaling(*lat_to);

    GPS_bearing(&GPS_FROM[LAT], &GPS_FROM[LON], &GPS_WP[LAT], &GPS_WP[LON], &target_bearing);
    GPS_distance_cm(&GPS_FROM[LAT], &GPS_FROM[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance);
#ifdef INS_PH_NAV_ON
    positionToHold[LAT] = (GPS_WP[LAT] - GPS_home[LAT]) * LAT_LON_TO_CM;
    positionToHold[LON] = (GPS_WP[LON] - GPS_home[LON]) * LAT_LON_TO_CM * GPS_scaleLonDown;
#else
    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_FROM[LAT], &GPS_FROM[LON]);
#endif
    waypoint_speed_gov = GPS_conf.nav_speed_min;
    original_target_bearing = target_bearing;
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp(void) {
    int32_t temp;
    temp = target_bearing - original_target_bearing;
    temp = wrap_18000(temp);
    return (abs(temp) > 10000);   // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision

void GPS_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* bearing) {
    int32_t off_x = *lon2 - *lon1;
    int32_t off_y = (*lat2 - *lat1) / GPS_scaleLonDown;

    *bearing = 9000 + atan2(-off_y, off_x) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

void GPS_distance_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, uint32_t* dist) {
    float dLat = (float) (*lat1 - *lat2);                                    // difference of latitude in 1/10 000 000 degrees
    float dLon = (float) (*lon1 - *lon2) * GPS_scaleLonDown; //x
    *dist = sqrt(sq(dLat) + sq(dLon)) * LAT_LON_TO_CM;
}

void calculateDistanceToHome(uint32_t* dist) {
	gpsDistanceToHome[LAT] = (GPS_coord[LAT] - GPS_home[LAT]) * LAT_LON_TO_CM;
	gpsDistanceToHome[LON] = (GPS_coord[LON] - GPS_home[LON]) * LAT_LON_TO_CM * GPS_scaleLonDown; //x
    *dist = sqrt(sq((float)gpsDistanceToHome[LAT]) + sq((float)gpsDistanceToHome[LON]));
}

//*******************************************************************************************************
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
//*******************************************************************************************************
static void GPS_calc_velocity(void) {
    static int16_t speed_old[2] = { 0, 0 };
    static int32_t last[2] = { 0, 0 };
    static uint8_t init = 0;

    if (init) {
    	// normalize GPS delta time for speed calculation (normalized time should not be used in PID controllers)
    	#define JITTER_DT	0.03f
    	float tmp;
    	if (dTnav >=  (HZ2S(10)-JITTER_DT) && dTnav <= (HZ2S(10)+JITTER_DT)) {
    		tmp = HZ2S(10);      // 10Hz Data 100ms
    	} else if (dTnav >= (HZ2S(5)-JITTER_DT) && dTnav <= (HZ2S(5)+JITTER_DT)) {
    		tmp = HZ2S(5);       //  5Hz Data 200ms
    	} else {
    		tmp = dTnav;
    	}
    	//debug[2] = tmp * 1000;

        tmp = 1.0 / tmp;
        gpsActualSpeed[_X] = (float) (GPS_coord[LON] - last[LON]) * GPS_scaleLonDown * tmp;
        gpsActualSpeed[_Y] = (float) (GPS_coord[LAT] - last[LAT]) * tmp;

        //TODO: Check unrealistic speed changes and signal navigation about possible gps signal degradation
		gpsActualSpeed[_X] = (gpsActualSpeed[_X] + speed_old[_X]) / 2;
		gpsActualSpeed[_Y] = (gpsActualSpeed[_Y] + speed_old[_Y]) / 2;

		speed_old[_X] = gpsActualSpeed[_X];
		speed_old[_Y] = gpsActualSpeed[_Y];

        //GPS_speed = 1.0f/InvSqrt( sq((int32_t)gpsActualSpeed[LAT]) + sq((int32_t)gpsActualSpeed[LON]) );
    }
    init = 1;

    last[LON] = GPS_coord[LON];
    last[LAT] = GPS_coord[LAT];
}

float getPredictedDeltaToStop(float *velocity) {
	//float timeToPredictPointInPH = conf.pid[PIDPOS].I8/100.0f;
	float tmp = (*velocity / 100.0f) * posholdPID_PARAM.kI;
	tmp = (tmp > 0) ? (tmp * tmp) : -(tmp * tmp);
	tmp = constrain(tmp, -15.0f, 15.0f); // in cm, i.e. +/-15 meters
	return tmp * 100.0f; // to cm
}

void predictAndSetPositionToHold() {

#ifdef INS_PH_NAV_ON

	//float timeToPredictPointInPH = conf.pid[PIDPOS].I8/100.0f;

	#ifdef DISABLE_INS_WHEN_PH_OFF
		positionToHold[LAT] = gpsDistanceToHome[LAT] + gpsActualSpeed[LAT] * posholdPID_PARAM.kI;
		positionToHold[LON] = gpsDistanceToHome[LON] + gpsActualSpeed[LON] * posholdPID_PARAM.kI;
	#else
		//positionToHold[LAT] = ins.positionEF[LAT] + getPredictedDeltaToStop(&ins.velocityEF[LAT]);
		//positionToHold[LON] = ins.positionEF[LON] + getPredictedDeltaToStop(&ins.velocityEF[LON]);
		positionToHold[LAT] = ins.positionEF[LAT] + ins.velocityEF[LAT] * posholdPID_PARAM.kI;
		positionToHold[LON] = ins.positionEF[LON] + ins.velocityEF[LON] * posholdPID_PARAM.kI;
	#endif
	GPS_hold[LAT] = GPS_home[LAT] + positionToHold[LAT]/LAT_LON_TO_CM;
	GPS_hold[LON] = GPS_home[LON] + positionToHold[LON]/(LAT_LON_TO_CM * GPS_scaleLonDown);
#else
	GPS_hold[LAT] = GPS_coord[LAT] + gpsActualSpeed[LAT] * posholdPID_PARAM.kI;
	GPS_hold[LON] = GPS_coord[LON] + ((gpsActualSpeed[LON] * posholdPID_PARAM.kI) / GPS_scaleLonDown);
	GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON], &GPS_hold[LAT], &GPS_hold[LON]); // hold at the predicted position
#endif
}


#ifndef INS_PH_NAV_ON

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//   100  = 1m
//  1000  = 11m    = 36 feet
//  1800  = 19.80m = 60 feet
//  3000  = 33m
// 10000  = 111m
//
static void GPS_calc_location_error(int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng) {
    gpsPositionError[LON] = (float) (*target_lng - *gps_lng) * GPS_scaleLonDown;  // X Error
    gpsPositionError[LAT] = *target_lat - *gps_lat; // Y Error
}

// TODO: check that the poshold target speed constraint can be increased for snappier poshold lock
static void applyGPSPosHoldPIDControl(float* dt) {
    int32_t d;
    int32_t target_speed;
    uint8_t axis;

    for (axis = 0; axis < 2; axis++) {
        target_speed = get_P(gpsPositionError[axis], &posholdPID_PARAM); // calculate desired speed from lat/lon error
        target_speed = constrain(target_speed, -100, 100);      // Constrain the target speed in poshold mode to 1m/s it helps avoid runaways..
        rate_error[axis] = target_speed - gpsActualSpeed[axis]; // calc the speed error

    nav[axis]      =
        get_P(rate_error[axis], &poshold_ratePID_PARAM)
                + get_I(rate_error[axis] + gpsPositionError[axis], dt, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

        d = get_D(gpsPositionError[axis], dt, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

        d = constrain(d, -2000, 2000);

        // get rid of noise
        if(abs(gpsActualSpeed[axis]) < 50) d = 0;

        nav[axis] += d;
        // nav[axis]  = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        nav[axis] = constrain_int16(nav[axis], -GPS_conf.nav_bank_max, GPS_conf.nav_bank_max);
        navPID[axis].integrator = poshold_ratePID[axis].integrator;
    }
}

#else

static void applyINSPosHoldPIDControl(float* dt) {

	int32_t target[2];
	uint8_t axis;
	for (axis = 0; axis < 2; axis++) {

		int32_t positionError = positionToHold[axis] - ins.positionEF[axis];
		int32_t targetSpeed = get_P(positionError, &posholdPID_PARAM); // calculate desired speed from lat/lon ins error
		targetSpeed = constrain(targetSpeed, -1000, 1000);      // Constrain the target speed in position hold mode to 10m/s

		int32_t rateError = targetSpeed - ins.velocityEF[axis]; // calculate the speed error
		rateError = constrain(rateError, -1000, 1000);
		nav[axis] = get_P(rateError, &poshold_ratePID_PARAM) +
					get_I(rateError, dt, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

		// rate D-part
		nav[axis] 	-= constrain_int16((ins.accelEF_Filtered[axis] * poshold_ratePID_PARAM.kD), -2000, 2000);

		//nav[axis]  = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
		nav[axis] = constrain_int16(nav[axis], -GPS_conf.nav_bank_max, GPS_conf.nav_bank_max);
		navPID[axis].integrator = poshold_ratePID[axis].integrator;
	}
}

#endif

void applyPosHoldPIDControl(float* dt) {
	// to avoid turnover, position hold have to be off when ground detected or takeoff,
	// because gps coordinate can be changed during the land detection
	if(!isTakeOffInProgress() && !isGroundDetectedFor100ms()) {
	  #ifdef INS_PH_NAV_ON
		applyINSPosHoldPIDControl(dt);
	  #else
		applyGPSPosHoldPIDControl(dt);
	  #endif

	} else {
		for (uint8_t i = 0; i < 2; i++) {
			nav[i] = 0;
		  #ifndef INS_PH_NAV_ON
			nav_rated[i] = 0;
		  #endif
			reset_PID(&poshold_ratePID[i]);
		}
	}
}

bool isNavStateForPosHold() {
	return 	   NAV_state == NAV_STATE_LAND_START
			|| NAV_state == NAV_STATE_LAND_SETTLE
			|| NAV_state == NAV_STATE_LAND_IN_PROGRESS
			|| NAV_state == NAV_STATE_HOLD_INFINIT
			|| NAV_state == NAV_STATE_HOLD_TIMED
			|| NAV_state == NAV_STATE_RTH_START
			|| NAV_state == NAV_STATE_WP_START;
}


////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH and WP
//
static void GPS_calc_nav_rate(uint16_t max_speed) {
    float trig[2];
    int32_t target_speed[2];
    int32_t tilt;
    uint8_t axis;

    GPS_update_crosstrack();
    int16_t cross_speed = crosstrack_error * (GPS_conf.crosstrack_gain / 100.0);  //check is it ok ?
    cross_speed = constrain(cross_speed, -200, 200);
    cross_speed = -cross_speed;

    float temp = (9000l - target_bearing) * RADX100;
    trig[_X] = cos(temp);
    trig[_Y] = sin(temp);

    target_speed[_X] = max_speed * trig[_X] - cross_speed * trig[_Y];
    target_speed[_Y] = cross_speed * trig[_X] + max_speed * trig[_Y];

    for (axis = 0; axis < 2; axis++) {
        rate_error[axis] = target_speed[axis] - gpsActualSpeed[axis];
        rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
    nav[axis]      =
        get_P(rate_error[axis],                        &navPID_PARAM)
       +get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM)
                + get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

        // nav[axis] = constrain(nav[axis],-NAV_BANK_MAX,NAV_BANK_MAX);
        nav[axis] = constrain_int16(nav[axis], -GPS_conf.nav_bank_max, GPS_conf.nav_bank_max);
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}

static void GPS_update_crosstrack(void) {
    // Crosstrack Error
    // ----------------
    // If we are too far off or too close we don't do track following
    float temp = (target_bearing - original_target_bearing) * RADX100;
    crosstrack_error = sin(temp) * wp_distance; // Meters we are off track line
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow
// speed rampup when starting a navigation
//
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
//                 |                                        +|+
//                 |< we should slow to 1 m/s as we hit the target
//
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow) {
    if (_slow) {
        max_speed = min(max_speed, wp_distance / 2);
    } else {
        max_speed = min(max_speed, wp_distance);
        max_speed = max(max_speed, GPS_conf.nav_speed_min);  // go at least nav_speed_min
    }
    // limit the ramp up of the speed
    // waypoint_speed_gov is reset to 0 at each new WP command
    if (max_speed > waypoint_speed_gov) {
        waypoint_speed_gov += (int) (100.0 * dTnav); // increase at .5/ms
        max_speed = waypoint_speed_gov;
    }
    return max_speed;
}

////////////////////////////////////////////////////////////////////////////////////
// Utilities
//

int32_t wrap_36000(int32_t ang) {
  if (ang > 36000) ang -= 36000;
  if (ang < 0)     ang += 36000;
    return ang;
}


/*
 * EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
 * with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
 * resolution also increased precision of nav calculations
 */

#define DIGIT_TO_VAL(_x)        (_x - '0')
uint32_t GPS_coord_to_degrees(char* s) {
    char *p, *q;
    uint8_t deg = 0, min = 0;
    unsigned int frac_min = 0;
    uint8_t i;

    // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++) ;
    q = s;

    // convert degrees
    while ((p - q) > 2) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }
    // convert minutes
    while (p > q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }
    // convert fractional minutes
    // expect up to four digits, result is in
    // ten-thousandths of a minute
    if (*p == '.') {
        q = p + 1;
        for (i = 0; i < 4; i++) {
            frac_min *= 10;
            if (isdigit(*q))
                frac_min += *q++ - '0';
        }
    }
    return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
}

// helper functions
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
	uint8_t i;
	uint16_t tmp = 0;

	for (i = 0; src[i] != 0; i++) {
		if (src[i] == '.') {
			i++;
			if (mult == 0)
				break;
			else
				src[i + mult] = 0;
		}
		tmp *= 10;
		if (src[i] >= '0' && src[i] <= '9')
			tmp += src[i] - '0';
	}
	return tmp;
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
    n -= '0';
  if(n>9)  n -= 7;
    n &= 0x0F;
    return n;
}

//************************************************************************
// Common GPS functions
//
void init_RTH() {
    f.GPS_mode = GPS_MODE_RTH;           // Set GPS_mode to RTH
    f.GPS_BARO_MODE = true;

    NAV_paused_at = 0;
    if (GPS_conf.rth_altitude == 0) {
        setAltToHold(alt.estAlt);     //Return at actual altitude
    } else {
        // RTH altitude is defined, but we use it only if we are below it
        if (alt.estAlt < GPS_conf.rth_altitude * 100) {
            setAltToHold(GPS_conf.rth_altitude * 100);
        } else {
            setAltToHold(alt.estAlt);
        }
    }
    f.GPS_head_set = 0;                                 //Allow the RTH ti handle heading

    // all RTH starts with a position hold. This allows to go to RTH altitude.
    predictAndSetPositionToHold(); // predict and set point to hold

    NAV_state = NAV_STATE_RTH_START;                    //NAV engine status is Starting RTH.
}

void GPS_reset_home_position(void) {
    if (f.GPS_FIX && GPS_numSat >= 5) {
        GPS_home[LAT] = GPS_coord[LAT];
        GPS_home[LON] = GPS_coord[LON];
        GPS_calc_longitude_scaling(GPS_coord[LAT]);    //need an initial value for distance and bearing calc
        nav_takeoff_bearing = att.heading;             //save takeoff heading
        //TODO: Set ground altitude
        f.GPS_FIX_HOME = 1;
    }
}

//reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav(void) {
    uint8_t i;

    for (i = 0; i < 2; i++) {
        nav[i] = 0;
	  #ifndef INS_PH_NAV_ON
		nav_rated[i] = 0;
	  #endif
        reset_PID(&posholdPID[i]);
        reset_PID(&poshold_ratePID[i]);
        reset_PID(&navPID[i]);
    }

    NAV_state = NAV_STATE_NONE;
    //invalidate JUMP counter
    jump_times = -10;
    //reset next step counter
    next_step = 1;
    //Clear poi
    GPS_poi[LAT] = 0;
    GPS_poi[LON] = 0;
    f.GPS_head_set = 0;
}

//Get the relevant P I D values and set the PID controllers
void GPS_set_pids(void) {
    posholdPID_PARAM.kP = (float) conf.pid[PIDPOS].P8 / 100.0;
    posholdPID_PARAM.kI = (float) conf.pid[PIDPOS].I8 / 100.0;
    posholdPID_PARAM.Imax = POSHOLD_IMAX * 100;

    poshold_ratePID_PARAM.kP = (float) conf.pid[PIDPOSR].P8 / 10.0;
    poshold_ratePID_PARAM.kI = (float) conf.pid[PIDPOSR].I8 / 100.0;
#ifdef INS_PH_NAV_ON
    poshold_ratePID_PARAM.kD = (float) conf.pid[PIDPOSR].D8 / 100.0;
#else
    poshold_ratePID_PARAM.kD = (float) conf.pid[PIDPOSR].D8 / 1000.0;
#endif
    poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    navPID_PARAM.kP = (float) conf.pid[PIDNAVR].P8 / 10.0;
    navPID_PARAM.kI = (float) conf.pid[PIDNAVR].I8 / 100.0;
    navPID_PARAM.kD = (float) conf.pid[PIDNAVR].D8 / 1000.0;
    navPID_PARAM.Imax = NAV_IMAX * 100;
}
//It was moved here since even i2cgps code needs it
int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
    return ang;
}




/**************************************************************************************/
/**************************************************************************************/
/***********************       specific  GPS device section  **************************/
/**************************************************************************************/
/**************************************************************************************/

#if defined(GPS_SERIAL)

/**************************************************************************************/
/***********************       NMEA                          **************************/
/**************************************************************************************/
#if defined(NMEA)
/* This is a light implementation of a GPS frame decoding
 This should work with most of modern GPS devices configured to output NMEA frames.
 It assumes there are some NMEA GGA frames to decode on the serial bus
 Here we use only the following data :
 - latitude
 - longitude
 - GPS fix is/is not ok
 - GPS num sat (4 is enough to be +/- reliable)
 - GPS altitude
 - GPS speed
 */
#define FRAME_GGA  1
#define FRAME_RMC  2

/*void GPS_SerialInit(void) {
    SerialOpen(GPS_SERIAL, GPS_BAUD);
    delay(1000);
}*/

bool GPS_newFrame(uint8_t c) {
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && (string[1] == 'P' || string[1] == 'N') && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && (string[1] == 'P' || string[1] == 'N') && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      if      (param == 2)                     {GPS_coord_temp[LAT] = GPS_coord_to_degrees(string);}
      else if (param == 3 && string[0] == 'S') GPS_coord_temp[LAT] = -GPS_coord_temp[LAT];
      else if (param == 4)                     {GPS_coord_temp[LON] = GPS_coord_to_degrees(string);}
      else if (param == 5 && string[0] == 'W') GPS_coord_temp[LON] = -GPS_coord_temp[LON];
      else if (param == 6)                     {f.GPS_FIX = (string[0]  > '0');}
      else if (param == 7)                     {GPS_numSat = grab_fields(string,0);}
      //else if (param == 8)                     {GPS_HDOP = grab_fields(string,0);}  // HDOP
      else if (param == 9)                     {GPS_altitude = grab_fields(string,0);}  // altitude in meters added by Mis
    } else if (frame == FRAME_RMC) {
      if      (param == 7)                     {GPS_speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
      else if (param == 8)                     {GPS_ground_course = grab_fields(string,1); }                 //ground course deg*10 
    }
    param++; offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    if (checksum_param) { //parity checksum
      uint8_t checksum = hex_c(string[0]);
      checksum <<= 4;
      checksum += hex_c(string[1]);
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
  } else {
     if (offset < 15) string[offset++] = c;
     if (!checksum_param) parity ^= c;
  }
  return frameOK && (frame==FRAME_GGA);
}
#endif //NMEA



/**************************************************************************************/
/***********************       UBLOX                         **************************/
/**************************************************************************************/
#if defined(UBLOX)
const char UBLOX_INIT[] PROGMEM = {                                                  // PROGMEM array must be outside any function !!!

	// disable all default NMEA messages
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19, // VGS: Course over ground and Ground speed
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15, // GSV: GNSS Satellites in View
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11, // GLL: Latitude and longitude, with time of position fix and status
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F, // GGA: Global positioning system fix data
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13, // GSA: GNSS DOP and Active Satellites
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17, // RMC: Recommended Minimum data

    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47, // set POSLLH MSG rate
//	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49, // set STATUS MSG rate
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F, // set SOL MSG rate
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67, // set VELNED MSG rate
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x00, 0x3B, 0xA2, // disable SVINFO
    
#ifdef UBLOX_GNSS_GPS_SBAS
    0xB5, 0x62, 0x06, 0x3E, 0x2C, 0x00, 0x00, 0x20, 0x20, 0x05, 0x00, // set GNSS to GPS + SBAS only 
    0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 
    0x01, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 
    0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x06, 0x08, 
    0x0E, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1A, 0x6F, 
#endif
#ifdef UBLOX_GNSS_GPS_SBAS_GLONASS
    0xB5, 0x62, 0x06, 0x3E, 0x2C, 0x00, 0x00, 0x20, 0x20, 0x05, 0x00, // set GNSS to GPS + SBAS + GLONASS 
    0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 
    0x01, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 
    0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x06, 0x08, 
    0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0x1C, 0x75,     
#endif          

#ifdef UBLOX_NAV5_PEDESTRIAN_MODEL
	0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, // CFG-NAV5 - Set engine settings (original MWII code)
	0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, // Collected by resetting a GPS unit to defaults. Changing mode to Pedistrian and
	0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, // capturing the data from the U-Center binary console.
	0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xC2,
#endif
#ifdef UBLOX_NAV5_AIRBORNE_1G_MODEL
	0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, // CFG-NAV5 - Set engine settings
	0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, // Airborne <1G
	0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
	0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x28,
#endif
#ifdef UBLOX_NAV5_AIRBORNE_4G_MODEL
	0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, // CFG-NAV5 - Set engine settings
	0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, // Airborne <4G
	0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
	0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x6C,
#endif

#ifdef UBLOX_SBAS_NONE
	0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x02, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0xC1, // set SBAS to NONE
#endif
#ifdef UBLOX_SBAS_AUTO
	0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0xC9, // set SBAS to AUTO
#endif
#ifdef UBLOX_SBAS_EGNOS
	0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x86, 0x25, // set SBAS to EGNOS for EUROPE
#endif
#ifdef UBLOX_SBAS_WAAS
	0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x04, 0xE0, 0x04, 0x00, 0x15, 0x81, // set SBAS to WAAS for USA/Canada
#endif
//	0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x02, 0x02, 0x00, 0x31, 0xD3, // set SBAS to MSAS
//	0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x80, 0x01, 0x00, 0x00, 0xAE, 0xCC, // set SBAS to GAGAN

#ifdef UBLOX_UPDATE_RATE_5HZ
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A // set rate to 5Hz
#endif
#ifdef UBLOX_UPDATE_RATE_10HZ
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12 // set rate to 10Hz
#endif
};

struct ubx_header {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
};
struct ubx_nav_posllh {
    uint32_t time;  // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
};
struct ubx_nav_solution {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
};
struct ubx_nav_velned {
    uint32_t time;  // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
};

enum ubs_protocol_bytes {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
};
enum ubs_nav_fix_type {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
};
enum ubx_nav_status_bits {
    NAV_STATUS_FIX_VALID = 1
};

// Receive buffer
static union {
    ubx_nav_posllh posllh;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    uint8_t bytes[];
}_buffer;

uint32_t init_speed[5] = {9600,19200,38400,57600,115200};

static void SerialGpsPrint(const char PROGMEM * str) {
    char b;
    while(str && (b = pgm_read_byte(str++))) {
        SerialWrite(GPS_SERIAL, b);
        delay(5);
    }
}

#define NEW_SP_END_UBLOX

void GPS_SerialInit(void) {
    SerialOpen(GPS_SERIAL,GPS_BAUD);
    delay(1000);
    for(uint8_t i=0;i<5;i++) {
        SerialOpen(GPS_SERIAL,init_speed[i]);          // switch UART speed for sending SET BAUDRATE command (NMEA mode)
#if (GPS_BAUD==19200)
        SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
#endif
#if (GPS_BAUD==38400)
        SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));     // 38400 baud
#endif
#if (GPS_BAUD==57600)
        SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));     // 57600 baud
#endif
#if (GPS_BAUD==115200)
        SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));    // 115200 baud
#endif

		#if defined(NEW_SP_END_UBLOX)
          while(!SerialTXfree(GPS_SERIAL)) delay(50);
        #else
          delay(50);
          SerialEnd(GPS_SERIAL);
          delay(50);
        #endif
    }
    delay(200);
    SerialOpen(GPS_SERIAL,GPS_BAUD);
    for(uint8_t i=0; i<sizeof(UBLOX_INIT); i++) {                        // send configuration data in UBX protocol
        SerialWrite(GPS_SERIAL, pgm_read_byte(UBLOX_INIT+i));
        delay(5);//simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
    }
}

bool GPS_newFrame(uint8_t data) {
    static uint8_t _step = 0; // State machine state
    static uint8_t _msg_id;
    static uint16_t _payload_length;
    static uint16_t _payload_counter;
    static uint8_t _ck_a;// Packet checksum accumulators
    static uint8_t _ck_b;

    uint8_t st = _step+1;
    bool ret = false;

    if (st == 2)
    if (PREAMBLE2 != data) st--;// in case of faillure of the 2nd header byte, still test the first byte
    if (st == 1) {
        if(PREAMBLE1 != data) st--;
    } else if (st == 3) { // CLASS byte, not used, assume it is CLASS_NAV
        _ck_b = _ck_a = data;// reset the checksum accumulators
    } else if (st > 3 && st < 8) {
        _ck_b += (_ck_a += data);  // checksum byte
        if (st == 4) {
            _msg_id = data;
        } else if (st == 5) {
            _payload_length = data; // payload length low byte
        } else if (st == 6) {
            _payload_length += (uint16_t)(data<<8);
            if (_payload_length > 512) st = 0;
            _payload_counter = 0;  // prepare to receive payload
        } else {
            if (_payload_counter+1 < _payload_length) st--; // stay in the same state while data inside the frame
            if (_payload_counter < sizeof(_buffer)) _buffer.bytes[_payload_counter] = data;
            _payload_counter++;
        }
    } else if (st == 8) {
        if (_ck_a != data) st = 0;  // bad checksum
    } else if (st == 9) {
        st = 0;
        if (_ck_b == data) { // good checksum
            if (_msg_id == MSG_POSLLH) {
                if(f.GPS_FIX) {
                    GPS_coord[LON] = _buffer.posllh.longitude;
                    GPS_coord[LAT] = _buffer.posllh.latitude;
                    GPS_altitude = _buffer.posllh.altitude_msl / 1000; //alt in m
                    //GPS_time       = _buffer.posllh.time; //not used for the moment
                }
                ret= true;        // POSLLH message received, allow blink GUI icon and LED, frame available for nav computation
            } else if (_msg_id == MSG_SOL) {
                f.GPS_FIX = 0;
                if((_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D || _buffer.solution.fix_type == FIX_2D)) f.GPS_FIX = 1;
                GPS_numSat = _buffer.solution.satellites;
            } else if (_msg_id == MSG_VELNED) {
                GPS_speed = _buffer.velned.speed_2d;  // cm/s
                GPS_ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000);// Heading 2D deg * 100000 rescaled to deg * 10 //not used for the moment
            }
        }
    }
    _step = st;
    return ret;
}
#endif //UBLOX



/**************************************************************************************/
/***********************       MTK                           **************************/
/**************************************************************************************/
#if defined(MTK_BINARY16) || defined(MTK_BINARY19) || defined(NMEA)

#define MTK_SET_BINARY          PSTR("$PGCMD,16,0,0,0,0,0*6A\r\n")
#define MTK_SET_NMEA            PSTR("$PGCMD,16,1,1,1,1,1*6B\r\n")
#define MTK_SET_NMEA_SENTENCES  PSTR("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n") // only GGA and RMC sentence
//#define MTK_SET_NMEA_SENTENCES  PSTR("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n") // only GGA sentence
#define MTK_OUTPUT_1HZ          PSTR("$PMTK220,1000*1F\r\n")
#define MTK_OUTPUT_4HZ          PSTR("$PMTK220,250*29\r\n")
#define MTK_OUTPUT_5HZ          PSTR("$PMTK220,200*2C\r\n")
#define MTK_OUTPUT_10HZ         PSTR("$PMTK220,100*2F\r\n")
#define MTK3333_OUTPUT_10HZ		PSTR("$PMTK300,100,0,0,0,0*2C\r\n")
#define MTK_NAVTHRES_OFF        PSTR("$PMTK397,0*23\r\n") // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s
#define MTK_NAVTHRES_OFF_3339   PSTR("$PMTK386,0*23\r\n") // Set Nav Threshold to zero for mtk3339 (according to PMTK_commands_A11.pdf)
#define SBAS_ON                 PSTR("$PMTK313,1*2E\r\n")
#define WAAS_ON                 PSTR("$PMTK301,2*2E\r\n")
#define SBAS_TEST_MODE          PSTR("$PMTK319,0*25\r\n")  //Enable test use of sbas satelite in test mode (usually PRN124 is in test mode)
#define SBAS_INTEGRITY_MODE     PSTR("$PMTK319,1*24\r\n") // Enable integrity mode of sbas satelite
#define DT_SBAS_TEST_MODE       PSTR("$PMTK519,0*23\r\n") // Enable test mode of sbas satelite (according to PMTK_commands_A11.pdf)
#define DT_SBAS_INTEGRITY_MODE  PSTR("$PMTK519,1*22\r\n") // Enable integrity mode of sbas satelite (according to PMTK_commands_A11.pdf)
#define DGPS_ON                 PSTR("$PMTK501,2*28\r\n") // set DGPS to WAAS (according to PMTK_commands_A11.pdf)
#define AIC_ON                  PSTR("$PMTK286,1*23\r\n") // only MTK3339 module: Active Interference Cancellation (AIC) feature provides effective narrow-band interference and jamming elimination.
#define EASY_ON                 PSTR("$PMTK869,1,1*35\r\n") // Enable EASY function (MT333x)
#define EASY_OFF                PSTR("$PMTK869,1,0*34\r\n") // Disable EASY function (MT333x)
#define LOCUS_SET_1SEC          PSTR("$PMTK187,1,1*3C\r\n") // only MTK3339 module: Set interval 1 sec for LOCUS (logging) mode.
#define LOCUS_START             PSTR("$PMTK185,0*22\r\n") // only MTK3339 module: Start logging in LOCUS mode.

#define MTK3333_ELEVATION_MASK	PSTR("$PMTK311,15*19\r\n") // only MTK3333 module and AXN3.80_8284_3333 firmware: set the Satellite elevation mask angel =15 degree.
#define MTK3333_NORMAL_MODE		PSTR("$PMTK886,0*28\r\n") // '0'=Normal mode: For general purpose
#define MTK3333_FITNESS_MODE	PSTR("$PMTK886,1*29\r\n") // '1'=Fitness mode: For running and walking purpose that the low-speed (<5m/s) movement will have more effect on the position calculation.
#define MTK3333_AVIATION_MODE	PSTR("$PMTK886,2*2A\r\n") // '2'=Aviation mode: For high-dynamic purpose that the large-acceleration movement will have more effect on the position calculation.
#define MTK3333_SET_SYNC_PPS	PSTR("$PMTK255,1*2D\r\n") // This message is used to enable or disable fix NMEA output time behind PPS function
#define MTK3333_SET_GPS_ONLY	PSTR("$PMTK353,1,0*36\r\n")
#define MTK3333_SET_GPS_AND_GLONASS	PSTR("$PMTK353,1,1*37\r\n")

struct diyd_mtk_msg {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    int32_t ground_speed;
    int32_t ground_course;
    uint8_t satellites;
    uint8_t fix_type;
    uint32_t utc_date;
    uint32_t utc_time;
    uint16_t hdop;
};

// #pragma pack(pop)
enum diyd_mtk_fix_type {
    FIX_NONE = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_2D_SBAS = 6,
    FIX_3D_SBAS = 7
};

#if defined(MTK_BINARY16)
enum diyd_mtk_protocol_bytes {
    PREAMBLE1 = 0xd0,
    PREAMBLE2 = 0xdd,
};
#endif

#if defined(MTK_BINARY19)
enum diyd_mtk_protocol_bytes {
    PREAMBLE1 = 0xd1,
    PREAMBLE2 = 0xdd,
};
#endif

// Packet checksum accumulators
uint8_t _ck_a;
uint8_t _ck_b;

// State machine state
uint8_t _step;
uint8_t _payload_counter;

// Time from UNIX Epoch offset
long _time_offset;
bool _offset_calculated;

// Receive buffer
union {
    diyd_mtk_msg msg;
    uint8_t bytes[];
}_buffer;

inline long _swapl(const void *bytes) {
    const uint8_t *b = (const uint8_t *)bytes;
    union {
        long v;
        uint8_t b[4];
    }u;

    u.b[0] = b[3];
    u.b[1] = b[2];
    u.b[2] = b[1];
    u.b[3] = b[0];

    return(u.v);
}

uint32_t init_speed[5] = {9600,19200,38400,57600,115200};

void SerialGpsPrint(const char PROGMEM * str) {
    char b;
    while(str && (b = pgm_read_byte(str++))) {
        SerialWrite(GPS_SERIAL, b);
    }
}

#define NEW_SP_END_MTK

void GPS_SerialInit(void) {

#if defined(INIT_MTK_GPS)	// MTK GPS setup

    for(uint8_t i=0;i<5;i++) {
        SerialOpen(GPS_SERIAL,init_speed[i]);                // switch UART speed for sending SET BAUDRATE command
#if (GPS_BAUD==19200)
        SerialGpsPrint(PSTR("$PMTK251,19200*22\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
#endif
#if (GPS_BAUD==38400)
        SerialGpsPrint(PSTR("$PMTK251,38400*27\r\n"));     // 38400 baud
#endif
#if (GPS_BAUD==57600)
        SerialGpsPrint(PSTR("$PMTK251,57600*2C\r\n"));     // 57600 baud
#endif
#if (GPS_BAUD==115200)
        SerialGpsPrint(PSTR("$PMTK251,115200*1F\r\n"));    // 115200 baud
#endif
        #if defined(NEW_SP_END_MTK)
          while(!SerialTXfree(GPS_SERIAL)) delay(80);
        #else
          delay(100);
          SerialEnd(GPS_SERIAL);
          delay(100);
        #endif
    }
    // at this point we have GPS working at selected (via #define GPS_BAUD) baudrate
    // So now we have to set the desired mode and update rate (which depends on the NMEA or MTK_BINARYxx settings)
    SerialOpen(GPS_SERIAL,GPS_BAUD);

    SerialGpsPrint(MTK_OUTPUT_1HZ);           // 1 Hz during tuning to avoid failure of configuration
		while(!SerialTXfree(GPS_SERIAL)) delay(80);

    SerialGpsPrint(MTK_SET_NMEA_SENTENCES);
		while(!SerialTXfree(GPS_SERIAL)) delay(80);

	//SerialGpsPrint(MTK3333_SET_GPS_ONLY);
	//SerialGpsPrint(MTK3333_SET_GPS_AND_GLONASS);
	//	while(!SerialTXfree(GPS_SERIAL)) delay(80);


    SerialGpsPrint(MTK_NAVTHRES_OFF);
    	while(!SerialTXfree(GPS_SERIAL)) delay(80);
    SerialGpsPrint(MTK_NAVTHRES_OFF_3339);
    	while(!SerialTXfree(GPS_SERIAL)) delay(80);

	#if defined(SET_SBAS_ENABLED)
        SerialGpsPrint(SBAS_ON);
          while(!SerialTXfree(GPS_SERIAL)) delay(80);

        SerialGpsPrint(WAAS_ON);
          while(!SerialTXfree(GPS_SERIAL)) delay(80);

		#ifdef SET_SBAS_TEST_MODE
        	SerialGpsPrint(SBAS_TEST_MODE);
		#else
        	SerialGpsPrint(SBAS_INTEGRITY_MODE); // integrity mode
		#endif
          while(!SerialTXfree(GPS_SERIAL)) delay(80);

        #ifdef SET_SBAS_TEST_MODE
          	SerialGpsPrint(DT_SBAS_TEST_MODE);
		#else
        	SerialGpsPrint(DT_SBAS_INTEGRITY_MODE); // dt integrity mode
		#endif
          while(!SerialTXfree(GPS_SERIAL)) delay(80);

        SerialGpsPrint(DGPS_ON);
          while(!SerialTXfree(GPS_SERIAL)) delay(80);
      #endif

      #if defined(MTK3339_AIC_ENABLED)
        SerialGpsPrint(AIC_ON); // The GPS signal could be recovered from jammed signal, and let user get better navigation quality.
          while(!SerialTXfree(GPS_SERIAL)) delay(80);
      #endif

	  #if defined(MTK3339_EASY_ENABLED)
        SerialGpsPrint(EASY_ON); // The GPS signal could be recovered from jammed signal, and let user get better navigation quality.
          while(!SerialTXfree(GPS_SERIAL)) delay(80);
      #endif

      #if defined(MTK3339_LOCUS_ENABLED)
        SerialGpsPrint(LOCUS_SET_1SEC);
          while(!SerialTXfree(GPS_SERIAL)) delay(80);
        SerialGpsPrint(LOCUS_START);
          while(!SerialTXfree(GPS_SERIAL)) delay(80);
      #endif


      //SerialGpsPrint(MTK3333_NORMAL_MODE);
      //SerialGpsPrint(MTK3333_FITNESS_MODE);
      SerialGpsPrint(MTK3333_AVIATION_MODE);
      	  while(!SerialTXfree(GPS_SERIAL)) delay(80);
      SerialGpsPrint(MTK3333_SET_SYNC_PPS);
       	  while(!SerialTXfree(GPS_SERIAL)) delay(80);

	  #if defined(GPS_NMEA_UPDATE_RATE_5HZ)
        SerialGpsPrint(MTK_OUTPUT_5HZ);           // 5 Hz update rate
        	while(!SerialTXfree(GPS_SERIAL)) delay(80);
      #endif
      #if defined(GPS_NMEA_UPDATE_RATE_10HZ)
		SerialGpsPrint(MTK_OUTPUT_10HZ);          // 10 Hz update rate
			while(!SerialTXfree(GPS_SERIAL)) delay(80);
		SerialGpsPrint(MTK3333_OUTPUT_10HZ);          // 10 Hz update rate
			while(!SerialTXfree(GPS_SERIAL)) delay(80);
      #endif


#if defined(MTK_BINARY19) || defined(MTK_BINARY16)
    SerialGpsPrint(MTK_SET_BINARY);
    	while(!SerialTXfree(GPS_SERIAL)) delay(80);
#endif


#else
    SerialOpen(GPS_SERIAL,GPS_BAUD);
    delay(500);
#endif  // init_mtk_gps
}

#if defined(MTK_BINARY19) || defined(MTK_BINARY16)
bool GPS_newFrame(uint8_t data) {
    bool parsed = false;

    restart:
    switch(_step) {
        // Message preamble, class, ID detection
        //
        // If we fail to match any of the expected bytes, we
        // reset the state machine and re-consider the failed
        // byte as the first byte of the preamble.  This
        // improves our chances of recovering from a mismatch
        // and makes it less likely that we will be fooled by
        // the preamble appearing as data in some other message.
        //
        case 0:
        if(PREAMBLE1 == data)
        _step++;
        break;
        case 1:
        if (PREAMBLE2 == data) {
            _step++;
            break;
        }
        _step = 0;
        goto restart;
        case 2:
        if (sizeof(_buffer) == data) {
            _step++;
            _ck_b = _ck_a = data;                  // reset the checksum accumulators
            _payload_counter = 0;
        } else {
            _step = 0;                             // reset and wait for a message of the right class
            goto restart;
        }
        break;
        // Receive message data
        case 3:
        _buffer.bytes[_payload_counter++] = data;
        _ck_b += (_ck_a += data);
        if (_payload_counter == sizeof(_buffer))
        _step++;
        break;
        // Checksum and message processing
        case 4:
        _step++;
        if (_ck_a != data)
        _step = 0;
        break;
        case 5:
        _step = 0;
        if (_ck_b != data)
        break;
        f.GPS_FIX = ((_buffer.msg.fix_type == FIX_3D) || (_buffer.msg.fix_type == FIX_3D_SBAS));
#if defined(MTK_BINARY16)
        GPS_coord_temp[LAT] = _buffer.msg.latitude * 10;    // XXX doc says *10e7 but device says otherwise
        GPS_coord_temp[LON] = _buffer.msg.longitude * 10;// XXX doc says *10e7 but device says otherwise
#endif
#if defined(MTK_BINARY19)
        GPS_coord_temp[LAT] = _buffer.msg.latitude;         // With 1.9 now we have real 10e7 precision
        GPS_coord_temp[LON] = _buffer.msg.longitude;
#endif
        GPS_altitude = _buffer.msg.altitude /100;    // altitude in meter
        GPS_speed = _buffer.msg.ground_speed;// in m/s * 100 == in cm/s
        GPS_ground_course = _buffer.msg.ground_course/100;//in degrees
        GPS_numSat = _buffer.msg.satellites;
        //GPS_time                    = _buffer.msg.utc_time;
        //GPS_hdop                  = _buffer.msg.hdop;
        parsed = true;
    }
    return parsed;
}
#endif

#endif //MTK

/**************************************************************************************/
/***********************       SkyTraq Venus838FLPx          **************************/
/************************      57600 baud                   ***************************/
/************************      update rate 5Hz              ***************************/
/**************************************************************************************/
#if defined(VENUS8)

// Input System Messages
#define VENUS_CONFIG_SERIAL_PORT          0x05
#define VENUS_CONFIG_OUTPUT_MSG_FORMAT    0x09
#define VENUS_CONFIG_POWER_MODE           0x0C
#define VENUS_CONFIG_GPS_UPDATE_RATE      0x0E

// Input GPS Messages
#define VENUS_CONFIG_GPS_PINNING          0x39
#define VENUS_CONFIG_GPS_PINNING_PARAMS   0x3B
#define VENUS_CONFIG_NAV_MODE             0x3C

// Output GPS Messages
#define VENUS_GPS_LOCATION                0xA8

// command messages
#define VENUS8_EXT2                       0x62
#define VENUS8_EXT3                       0x63
#define VENUS8_EXT4                       0x64
#define VENUS8_CONFIG_SBAS                0x01  // w/EXT2
#define VENUS8_CONFIG_INTERFERENCE_DETECT 0x06  // w/EXT4
#define VENUS8_CONFIG_NAV_MODE            0x17  // w/EXT4
#define VENUS8_CONFIG_SAGPS               0x01  // w/EXT3

typedef struct {
    int32_t x,y,z;
}xyz32_t;

typedef struct {
    uint8_t fixmode;
    uint8_t sv_count;  // satellites
    uint16_t gps_week;
    uint32_t gps_tow;
    int32_t latitude;
    int32_t longitude;
    uint32_t ellipsoid_alt;
    uint32_t sealevel_alt;
    uint16_t gdop, pdop, hdop, vdop, tdop;
    xyz32_t ecef,vel;
}venus_location;

typedef struct {
    uint8_t id;     // message id
    union {
        uint8_t body[];
        venus_location location;
    };
}venus_message;


typedef struct {
    union {
        uint8_t payload[];
        venus_message message;
    };
}venus_payload;

static venus_payload venus_ctx;

#define SWAP16(x) ((x&0xff)<<8 | (x>>8))
#define SWAP32(x) ((x&0xff)<<24 | ((x&0xff00)<<8) | ((x&0xff0000)>>8) | ((x&0xff000000)>>24))

void VenusFixLocationEndianess() { // we only do relevant ones to save CPU time
    venus_ctx.message.location.latitude = SWAP32(venus_ctx.message.location.latitude);
    venus_ctx.message.location.longitude = SWAP32(venus_ctx.message.location.longitude);
    venus_ctx.message.location.sealevel_alt = SWAP32(venus_ctx.message.location.sealevel_alt);
}

void venusWrite(uint8_t length) {
    uint8_t pls;
    uint8_t cs=0;
    SerialWrite(GPS_SERIAL,0xA0);
    SerialWrite(GPS_SERIAL,0xA1);
    SerialWrite(GPS_SERIAL,0); // length is never higher than 8 bits
    SerialWrite(GPS_SERIAL,length);
    for(pls=0; pls<length; pls++) {
        cs = cs ^ venus_ctx.payload[pls];
        SerialWrite(GPS_SERIAL,venus_ctx.payload[pls]);
    }
    SerialWrite(GPS_SERIAL,cs);
    SerialWrite(GPS_SERIAL,0x0D);
    SerialWrite(GPS_SERIAL,0x0A);
    delay(50);
}

void GPS_SerialInit(void) {
    uint32_t init_speed[5] = {9600,19200,38400,115200,57600};

    for(uint8_t i=0;i<5;i++) {
        SerialOpen(GPS_SERIAL,init_speed[i]);

        venus_ctx.message.id = VENUS_CONFIG_OUTPUT_MSG_FORMAT;
        venus_ctx.message.body[0] = 2; // VENUS BINARY
        venus_ctx.message.body[1] = 0;// SRAM
        venusWrite(3);// message payload length = 3

        venus_ctx.message.id = VENUS_CONFIG_SERIAL_PORT;
        venus_ctx.message.body[0] = 0;// Venus device's COM1
        venus_ctx.message.body[1] = 4;// 57600
        venus_ctx.message.body[2] = 0;// SRAM
        venusWrite(4);
    }
    delay(200);

    // configure NAV MODE
    venus_ctx.message.id = VENUS8_EXT4;
    venus_ctx.message.body[0] = VENUS8_CONFIG_NAV_MODE;
    venus_ctx.message.body[1] = 5;// NAVMODE AUTO
    venus_ctx.message.body[2] = 0;// SRAM
    venusWrite(4);

    // configure INTERFERENCE DETECTION
    venus_ctx.message.id = VENUS8_EXT4;
    venus_ctx.message.body[0] = VENUS8_CONFIG_INTERFERENCE_DETECT;
    venus_ctx.message.body[1] = 1;// enable
    venus_ctx.message.body[2] = 0;// SRAM
    venusWrite(4);

    // configure INTERFERENCE DETECTION
    venus_ctx.message.id = VENUS8_EXT2;
    venus_ctx.message.body[0] = VENUS8_CONFIG_SBAS;
    venus_ctx.message.body[1] = 1;// SBAS enable
    venus_ctx.message.body[2] = 1;// use SBAS satellite for navigation
    venus_ctx.message.body[3] = 8;// default range
    venus_ctx.message.body[4] = 1;// enable the correction
    venus_ctx.message.body[5] = 3;// number of tracking channels
    venus_ctx.message.body[6] = 7;// Allows WAAS / EGNOS / MSAS
    venus_ctx.message.body[7] = 0;// SRAM
    venusWrite(9);

    // disable POSITION PINNING
    venus_ctx.message.id = VENUS_CONFIG_GPS_PINNING;
    venus_ctx.message.body[0] = 2;// POSPINNING DISABLE
    venus_ctx.message.body[1] = 0;// SRAM
    venusWrite(3);

    // disable POSITION PINNING
    venus_ctx.message.id = VENUS_CONFIG_GPS_PINNING_PARAMS;
    for(uint8_t i=0;i<10;i++) venus_ctx.message.body[i] = 0;
    venus_ctx.message.body[10] = 0;// SRAM
    venusWrite(12);

    // disable SAGPS
    venus_ctx.message.id = VENUS8_EXT3;
    venus_ctx.message.body[0] = VENUS8_CONFIG_SAGPS;
    venus_ctx.message.body[1] = 2;// SAGPS disable
    venus_ctx.message.body[2] = 0;// SRAM
    venusWrite(4);

    // NORMAL power mode
    venus_ctx.message.id = VENUS_CONFIG_POWER_MODE;
    venus_ctx.message.body[0] = 0;// POWERMODE NORMAL
    venus_ctx.message.body[1] = 0;// SRAM
    venusWrite(3);

    // enable the update rate
    venus_ctx.message.id = VENUS_CONFIG_GPS_UPDATE_RATE;
    venus_ctx.message.body[0] = 5;// VENUS UPDATE RATE
    venus_ctx.message.body[1] = 0;// SRAM
    venusWrite(3);
}

bool GPS_newFrame(uint8_t c) {
    static uint8_t state=0;
    static uint8_t n=0;
    static uint8_t cr=0;
    static uint8_t length; // payload length
    bool ret = false;

    switch(state) {
        case 0: if(c==0xA0) state++; break;
        case 1: if(c==0xA1) state++; else state=0; break;
        case 2: state++; break;  // first byte of length is always 0 because message payload is never higher than 255
        case 3: length=c; state++; break;// according to the spec, length is always >1
        case 4:
        venus_ctx.message.id = c;
        cr=c;
        n=1;
        state++;
        break;
        case 5:// read bytes of the payload
        cr ^= c;// adjust checksum
        if(n<sizeof(venus_ctx.message))
        venus_ctx.payload[n] = c;
        n++;
        if(n==length) state++;
        break;
        case 6:
        if(c==cr) state++;
        else state=0;// bad cr
        break;// check checksum, abort if not-equal
        case 7:
        if(c==0x0D) state++;
        else state=0;
        break;
        case 8:
        state=0;
        if(c!=0x0A) break;
        if(venus_ctx.message.id==VENUS_GPS_LOCATION) {
            GPS_numSat = venus_ctx.message.location.sv_count;
            f.GPS_FIX = venus_ctx.message.location.fixmode >=2;
            if (f.GPS_FIX) {
                VenusFixLocationEndianess();
                GPS_coord[LAT] = venus_ctx.message.location.latitude;
                GPS_coord[LON] = venus_ctx.message.location.longitude;
                GPS_altitude = venus_ctx.message.location.sealevel_alt /100;    // altitude in meter
            }
            ret=true;
        }
    }
    return ret;
}
#endif

#endif //GPS SERIAL




/**************************************************************************************/
/***************                       I2C GPS                     ********************/
/**************************************************************************************/
#if defined(I2C_GPS)
#define I2C_GPS_ADDRESS               0x20 //7 bits

#define I2C_GPS_STATUS_00             00    //(Read only)
#define I2C_GPS_STATUS_NEW_DATA       0x01  // New data is available (after every GGA frame)
#define I2C_GPS_STATUS_2DFIX          0x02  // 2dfix achieved
#define I2C_GPS_STATUS_3DFIX          0x04  // 3dfix achieved
#define I2C_GPS_STATUS_NUMSATS        0xF0  // Number of sats in view
#define I2C_GPS_LOCATION              07    // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_GROUND_SPEED          31    // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE              33    // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE         35    // GPS ground course (uint16_t)
#define I2C_GPS_TIME                  39    // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
#define I2C_GPS_SONAR_ALT             239   // Sonar Altitude

uint8_t GPS_NewData(void) {
    uint8_t i2c_gps_status;

    i2c_gps_status = i2c_readReg(I2C_GPS_ADDRESS,I2C_GPS_STATUS_00);                 //Get status register

#if defined(I2C_GPS_SONAR)
    i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_SONAR_ALT, (uint8_t*)&sonarAlt,2);
#endif

    f.GPS_FIX = 0;
    if (i2c_gps_status & I2C_GPS_STATUS_3DFIX) {                                     //Check is we have a good 3d fix (numsats>5)
        f.GPS_FIX = 1;
        if (i2c_gps_status & I2C_GPS_STATUS_NEW_DATA) {                                //Check about new data
            GPS_Frame = 1;
            if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;//Blink GPS update
            GPS_numSat = i2c_gps_status >>4;
            i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_LOCATION, (uint8_t*)&GPS_coord[LAT],4);
            i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_LOCATION+4, (uint8_t*)&GPS_coord[LON],4);
            // note: the following vars are currently not used in nav code -- avoid retrieving it to save time
            //i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_GROUND_SPEED, (uint8_t*)&GPS_speed,2);
            //i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_ALTITUDE,     (uint8_t*)&GPS_altitude,2);
            //i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_GROUND_COURSE,(uint8_t*)&GPS_ground_course,2);
            return 1;
        }
    }
    return 0;
}
#endif //I2C_GPS



#endif // GPS Defined
