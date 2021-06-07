#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MahoWii.h"
#include "AltHold.h"

#if BARO

#define ALT_HOLD_CONTROL_UPDATE_RATE   50		// 50hz update rate

int32_t altToHold; // in cm
int16_t hoveringThrottle = 0;
int32_t targetVario = 0;
bool takeOffInProgress = false;

bool applyAltHoldControl() {

	static timer_t altHoldControlTimer;

    if (updateTimer(&altHoldControlTimer, HZ2US(ALT_HOLD_CONTROL_UPDATE_RATE))) {

    	//debug[3] = altHoldControlTimer.dTime/10;

    	static bool isBaroModeActivated = false;
		static bool isHoveringState = false;

		if ((f.BARO_MODE
				#if GPS
					|| f.GPS_BARO_MODE
				#endif
				) && f.ARMED) {

			if (!isBaroModeActivated) {
				isBaroModeActivated = true;
				isHoveringState = false;
				takeOffInProgress = false;

				initHoveringThrottle();
				resetLandDetector();
			}

			runLandDetector();

	#if GPS
			if (f.GPS_BARO_MODE) {

				if (takeOffInProgress) {
					takeOffInProgress = false;
				}

				if (	   NAV_state == NAV_STATE_LAND_IN_PROGRESS
						|| NAV_state == NAV_STATE_LAND_DETECTED
						|| NAV_state == NAV_STATE_LANDED) {

					if (isHoveringState) {
						isHoveringState = false;
					}

					setAltToHold(alt.estAlt); // for safety, e.g. if NAV_state jump to NAV_STATE_NONE when numSat dropped below 4 during navigation

					targetVario = constrain(GPS_conf.min_nav_vario, 30, 100);
					if (alt.estAlt > SAFE_NAV_ALTITUDE) {
						targetVario += (int32_t)(MAX_NAV_VARIO - GPS_conf.min_nav_vario)
								* (alt.estAlt - SAFE_NAV_ALTITUDE) / (GPS_conf.rth_altitude*100 - SAFE_NAV_ALTITUDE);
					}
					targetVario = -targetVario; // for landing target vario is negative

				} else {

					if (!isHoveringState) {
						isHoveringState = true;
					}

					targetVario = ((altToHold - alt.estAlt) * 3) / 2;
					if (alt.estAlt > SAFE_NAV_ALTITUDE) {
						targetVario = constrain(targetVario, -MAX_NAV_VARIO, MAX_NAV_VARIO);
					} else { // reduce desired speed if altitude less then SAFE_NAV_ALTITUDE
						targetVario = constrain(targetVario, -(MAX_NAV_VARIO/3), (MAX_NAV_VARIO/3));
					}
				}
		#ifdef BUZZER
				beepBuzzer(targetVario);
		#endif

			} else { // here f.BARO_MODE is always true

	#endif	// end #if GPS

				int16_t throttleDiff = rcData[THROTTLE] - MIDRC;

				if (!takeOffInProgress) {
					if ((rcData[THROTTLE] < MINCHECK) /*&& (alt.EstAlt < MIN_BARO_TRUSTED_ALTITUDE)*/) {
						if (isGroundDetectedFor100ms()) {
							takeOffInProgress = true;
						}
					}
				} else {
					if ((throttleDiff > ALT_HOLD_THROTTLE_NEUTRAL_ZONE) && (alt.estVario >= 15)) {
						takeOffInProgress = false; // reset when vario >= 15
					}
				}

				if (takeOffInProgress || // it means that we are going to take off, so it necessary to skip first AH case to avoid jumps near the ground
		#ifdef SAFE_ALT_DURING_AH
						(!f.SAFE_ALT_MODE && (abs(throttleDiff) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE))
						|| (f.SAFE_ALT_MODE
								&& ((throttleDiff > ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
										|| ((throttleDiff < -ALT_HOLD_THROTTLE_NEUTRAL_ZONE) && (alt.estAlt > SAFE_ALT_DURING_AH))))
		#else
						(abs(throttleDiff) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
		#endif
						) {

					if (isHoveringState) {
						isHoveringState = false;
					}

					if (abs(throttleDiff) <= ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
						targetVario = 0;
					} else {
						targetVario = ((throttleDiff - ((throttleDiff > 0) ? ALT_HOLD_THROTTLE_NEUTRAL_ZONE : -ALT_HOLD_THROTTLE_NEUTRAL_ZONE)) * 3)/ 4;
					}

		#ifdef BUZZER
					beepBuzzer(targetVario);
		#endif

				} else { // Alt hold activated, hovering state, throttle at middle neutral zone

					if (!isHoveringState) {
						isHoveringState = true;

		#ifdef SAFE_ALT_DURING_AH
						altToHold = (f.SAFE_ALT_MODE && alt.estAlt < SAFE_ALT_DURING_AH) ? SAFE_ALT_DURING_AH : alt.estAlt;
		#else
						altToHold = alt.estAlt;
		#endif

		#ifdef BUZZER
						stopBuzzer();
		#endif
					}

					targetVario = ((altToHold - alt.estAlt) * 3) / 2;
				}
	#if GPS
			}
	#endif
			applyPIDControl(altHoldControlTimer.dTime, isHoveringState);

			return true;

		} else {
			if (isBaroModeActivated) {
				isBaroModeActivated = false;
				takeOffInProgress = false;
				targetVario = 0;
				resetVarioErrorIPart();
				resetLandDetector();

			  #ifdef BUZZER
				stopBuzzer();
			  #endif
			}
		}

    }

    return false;
}

bool isTakeOffInProgress() {
	return takeOffInProgress;
}


//#define VARIO_P 48      // it's force to get desired vario
//#define VARIO_I 20
//#define VARIO_D 16      // regulate the speed of vario change and prevent oscillations of PID controller, e.g. if VARIO_D=0 it means that speed to get desired vario (by throttle stick) will be max

#define VARIO_ERROR_I_PART_MAX		250
#define VARIO_ERROR_I_SUM_RANK		16
#define VARIO_ERROR_I_SUM_MAX		16384000 // = VARIO_ERROR_I_PART_MAX * (2 ^ VARIO_ERROR_I_SUM_RANK)
int32_t varioErrorISum = 0;
int16_t varioErrorIPart = 0;

void applyPIDControl(uint16_t dTime, bool isHoveringState) {

	targetVario = constrain(targetVario, -MAX_VARIO, MAX_VARIO); // desired speed/vario in limit of +/-MAX_VARIO

    debug[0] = targetVario;

    int32_t varioError = targetVario - alt.estVario;
    varioError = constrain(varioError, -600, 600);

    // reduce speed of I-part at hovering to avoid wobble near the point
    varioErrorISum += ((varioError * conf.pid[PIDALT].I8 * dTime) >> 7)/((isHoveringState && abs(targetVario) < 100) ? 2 : 1);
    varioErrorISum = constrain(varioErrorISum, -VARIO_ERROR_I_SUM_MAX, VARIO_ERROR_I_SUM_MAX);

    varioErrorIPart = (varioErrorISum >> VARIO_ERROR_I_SUM_RANK); // should be in range +/-VARIO_ERROR_I_PART_MAX ! land detector depends on this!
    varioErrorIPart = constrain(varioErrorIPart, -VARIO_ERROR_I_PART_MAX, VARIO_ERROR_I_PART_MAX);

    int16_t varioPIDControl = ((varioError * conf.pid[PIDALT].P8) >> 5)
								+ varioErrorIPart
								- (((int32_t)ins.accelEF_Filtered[ALT] * conf.pid[PIDALT].D8) >> 6);
    // varioPIDControl = constrain(baroPID, -600, 600);

    rcCommand[THROTTLE] = hoveringThrottle + varioPIDControl;
    rcCommand[THROTTLE] = constrain(rcCommand[THROTTLE], conf.minthrottle + 50, MAXTHROTTLE - 50);
}

void resetVarioErrorIPart() {
	varioErrorISum = constrain(varioErrorISum, -VARIO_ERROR_I_SUM_MAX/2, VARIO_ERROR_I_SUM_MAX/2);
	varioErrorIPart = constrain(varioErrorIPart, -VARIO_ERROR_I_PART_MAX/2, VARIO_ERROR_I_PART_MAX/2);
}


#define HOVERING_THROTTLE_DELTA    250

void initHoveringThrottle() {
    // Correct hovering throttle on AH activation if it's out of (MIDRC-250, MIDRC+250) range.
    // It will be true only for the first AH activation after power ON or in case of hovering throttle out of range
    if ((hoveringThrottle < (MIDRC - HOVERING_THROTTLE_DELTA)) || (hoveringThrottle > (MIDRC + HOVERING_THROTTLE_DELTA))) {
#ifdef HOVERING_THROTTLE
        hoveringThrottle = HOVERING_THROTTLE;
#else
        uint16_t tmp = conf.thrMid8 / 10;
        hoveringThrottle = lookupThrottleRC[tmp] + (conf.thrMid8 % 10) * (lookupThrottleRC[tmp + 1] - lookupThrottleRC[tmp]) / 10;
#endif
        hoveringThrottle = constrain(hoveringThrottle, MIDRC - HOVERING_THROTTLE_DELTA, MIDRC + HOVERING_THROTTLE_DELTA);
    }
}

#if GPS
void setAltToHold(int32_t newAltToHold) {
    //Limit maximum altitude command
    if (newAltToHold > GPS_conf.nav_max_altitude * 100) {
        newAltToHold = GPS_conf.nav_max_altitude * 100;
    }

    // save the target altitude
    altToHold = newAltToHold;
}

#define TARGET_ALT_DELTA    50

bool isAltitudeReached() {
    return abs(altToHold - alt.estAlt) < TARGET_ALT_DELTA;
}
#endif

// depends on range of I part. varioErrorIPart reach this if we landed (see applyPIDControl() for details).
// Here -10 value to ignore vibro fluctuations.
#define VARIO_ERROR_I_PART_LAND_BOUND    (VARIO_ERROR_I_PART_MAX - 15)

#define TIME_TO_CATCH_RAW_GROUND_ALT	100		// in ms
#define TIME_TO_BE_SHURE_DRONE_LANDED	4000	// in ms

bool groundRawAltSet = false;
uint32_t landDetectorStartTime;
uint32_t timeOnLand;

void runLandDetector() {

	if (isGroundDetected()) {
        timeOnLand = millis() - landDetectorStartTime;
    } else {
        // we've detected movement up or down so reset land detector
    	resetLandDetector();
    }

	// protect to update groundRawAlt to keep 1st registration of ground altitude after 100ms, until reset, i.e. landDetected=false
	if (!groundRawAltSet && (timeOnLand >= TIME_TO_CATCH_RAW_GROUND_ALT)) {
		groundRawAltSet = true;
		alt.groundRawAlt = alt.rawAlt;
	}
}

void resetLandDetector() {
	landDetectorStartTime = millis();
	timeOnLand = 0;
	groundRawAltSet = false;
}

// fast detector: detect whether we have landed by watching for low climb rate and VARIO_ERROR_I_PART_LAND_BOUND
bool isGroundDetected() {
	return (abs(alt.estVario) < 15) && (varioErrorIPart <= -VARIO_ERROR_I_PART_LAND_BOUND)
						&& (alt.estAlt < SAFE_NAV_ALTITUDE);
}

// true if groundRawAlt is set and ground detected for at least TIME_TO_CATCH_RAW_GROUND_ALT
bool isGroundDetectedFor100ms() {
	return groundRawAltSet;
}

bool isLanded() {
	return groundRawAltSet && (timeOnLand >= TIME_TO_BE_SHURE_DRONE_LANDED);
}


#ifdef BUZZER
void beepBuzzer(int16_t targetVario) {
    static uint32_t varioBuzzerTime;
    static uint8_t buzzerCount;
	if (currentTime > varioBuzzerTime) {
		if (++buzzerCount > (abs(targetVario) / 30) + 10) {
			buzzerCount = 0;
		}
		varioBuzzerTime = currentTime + 40000;
		if (buzzerCount >= 10 && ((buzzerCount % 2) == 0)) {
			BUZZERPIN_ON
		} else {
			BUZZERPIN_OFF
		}
	}
}

void inline stopBuzzer() {
    BUZZERPIN_OFF
}
#endif


#endif //BARO
