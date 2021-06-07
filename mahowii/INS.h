#ifndef INS_H_
#define INS_H_


extern float cy, sy;

void resetXYState();

void correctXYStateWithGPS(float* dt);

void updateXYState(float* dt);

void saveXYPositionToHistory();

void resetZState();

void correctZStateWithBaro(float* dt);

void updateZState(float* dt);

void saveZPositionToHistory();

bool calculateXY_INS();

bool calculateZ_INS();

void rotateAccelToEarthFrame();

void calculateRAWAltitude();

//void calculateRAWVario(float* dt);

void updateAccelEF_Filtered(uint8_t axis);

#endif /* INS_H_ */
