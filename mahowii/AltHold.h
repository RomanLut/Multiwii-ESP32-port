#ifndef ALT_HOLD_H_
#define ALT_HOLD_H_


extern int32_t targetVario;
extern int32_t altToHold;
extern int16_t fwAltPID[4];// P,I,D,Output

#if BARO

bool applyAltHoldControl();

void setAltToHold(int32_t newAltToHold);

bool isAltitudeReached();

bool isTakeOffInProgress();
bool isLandDetected();
void resetVarioErrorIPart();

void runLandDetector();
void resetLandDetector();
bool isGroundDetected();
bool isGroundDetectedFor100ms();
bool isLanded();

void initHoveringThrottle();

void applyPIDControl(uint16_t dTime, bool isHoveringState);

#ifdef BUZZER
    void beepBuzzer(int16_t targetVario);
    void stopBuzzer();
#endif


#endif

#endif /* ALT_HOLD_H_ */
