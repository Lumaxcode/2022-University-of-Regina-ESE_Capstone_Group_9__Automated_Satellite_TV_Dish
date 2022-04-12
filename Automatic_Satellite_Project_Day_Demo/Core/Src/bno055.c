#include "bno055.h"
#include <string.h>
#include <math.h>
uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 1;
uint16_t quaScale = (1<<14);    // 2^14
double CMPS=0;


tilt_YX normalized_azimuth(double xm, double ym, double zm, double thetaR, double phiR ){
			tilt_YX tilt;
	    tilt.Xm=(xm)*cos(thetaR)-
   				(ym)*sin(phiR)*sin(thetaR)
 					+zm*cos(phiR)*sin(thetaR);
 			tilt.Ym=(ym)*cos(phiR)+(zm)*sin(phiR);
	   tilt.Azimuth=tiltcompensated_azimuth(tilt.Xm, tilt.Ym);
	 return tilt;
	
}
Radians theta_phi_rad(double ax, double ay, double az){ 
			Radians rad;
			rad.phi=(-atan2(ay/9.8,az/9.8)/2/3.141592654*360)/360*(2*3.14); 
			rad.theta=(-atan2(ax/9.8,az/9.8)/2/3.141592654*360)/360*(2*3.14);
			return rad;
}

double tiltcompensated_azimuth(double X,double Y){
  
if((X > 0)&&(Y > 0))
				{
				CMPS = atan(Y/X)*57.3;}
			else if((X > 0)&&(Y < 0))
				{CMPS= 360+atan(Y/X)*57.3;}
			else if((X == 0)&&(Y > 0))
				{CMPS = 90;}
			else if((X == 0)&&(Y < 0)) 
				{CMPS = 270;}
			else if(X < 0) 
				{CMPS = 180+atan(Y/X)*57.3;}

return CMPS;

}
void bno055_setPage(uint8_t page) { bno055_writeData(BNO055_PAGE_ID, page); }

bno055_opmode_t bno055_getOperationMode(void) {
  bno055_opmode_t mode;
  bno055_readData(BNO055_OPR_MODE, &mode, 1);
  return mode;
}


void bno055_setOperationMode(bno055_opmode_t mode) {
  bno055_writeData(BNO055_OPR_MODE, mode);
  if (mode == BNO055_OPERATION_MODE_CONFIG) {
    bno055_delay(19);
  } else {
    bno055_delay(7);
  }
}

void bno055_setOperationModeConfig(void) {
  bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF(void) {
  bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF); 
}
void bno055_setOperationModeCOMPASS(void) { 
	bno055_setOperationMode(BNO055_OPERATION_MODE_COMPASS);
}
void bno055_setOperationModeM4G(void) { 
	bno055_setOperationMode(BNO055_OPERATION_MODE_M4G);
}
 

void bno055_setExternalCrystalUse(bool state) {
  bno055_setPage(0);
  uint8_t tmp = 0;
  bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
  tmp |= (state == true) ? 0x80 : 0x0;
  bno055_writeData(BNO055_SYS_TRIGGER, tmp);
  bno055_delay(700);
}

void bno055_enableExternalCrystal(void) { bno055_setExternalCrystalUse(true); }
void bno055_disableExternalCrystal(void) { bno055_setExternalCrystalUse(false); }

void bno055_reset(void) {
  bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
  bno055_delay(700);
}

int8_t bno055_getTemp(void) {
  bno055_setPage(0);
  uint8_t t;
  bno055_readData(BNO055_TEMP, &t, 1);
  return t;
}

void bno055_setup(void) {
  bno055_reset();

  uint8_t id = 0;
  bno055_readData(BNO055_CHIP_ID, &id, 1);
    if (id != BNO055_ID) {
      printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
  }
  bno055_setPage(0);
  bno055_writeData(BNO055_SYS_TRIGGER, 0x0);

  // Select BNO055 config mode
  bno055_setOperationModeConfig();
  bno055_delay(10);
}

int16_t bno055_getSWRevision(void) {
  bno055_setPage(0);
  uint8_t buffer[2];
  bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
  return (int16_t)((buffer[1] << 8) | buffer[0]);
}

uint8_t bno055_getBootloaderRevision(void) {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
  return tmp;
}

uint8_t bno055_getSystemStatus(void) {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
  return tmp;
}

bno055_self_test_result_t bno055_getSelfTestResult(void) {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_self_test_result_t res = {
      .mcuState = 0, .gyrState = 0, .magState = 0, .accState = 0};
  bno055_readData(BNO055_ST_RESULT, &tmp, 1);
  res.mcuState = (tmp >> 3) & 0x01;
  res.gyrState = (tmp >> 2) & 0x01;
  res.magState = (tmp >> 1) & 0x01;
  res.accState = (tmp >> 0) & 0x01;
  return res;
}

uint8_t bno055_getSystemError(void) {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

bno055_calibration_state_t bno055_getCalibrationState(void) {
  bno055_setPage(0);
  bno055_calibration_state_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
  uint8_t calState = 0;
  bno055_readData(BNO055_CALIB_STAT, &calState, 1);
  cal.sys = (calState >> 6) & 0x03;
  cal.gyro = (calState >> 4) & 0x03;
  cal.accel = (calState >> 2) & 0x03;
  cal.mag = calState & 0x03;
  return cal;
}


bno055_calibration_data_t bno055_getCalibrationData(void) {
  bno055_calibration_data_t calData;
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

  // Assumes little endian processor
  memcpy(&calData.offset.accel, buffer, 6);
  memcpy(&calData.offset.mag, buffer + 6, 6);
  memcpy(&calData.offset.gyro, buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag, buffer + 20, 2);

  bno055_setOperationMode(operationMode);

  return calData;
}

void bno055_setCalibrationData(bno055_calibration_data_t calData) {
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  // Assumes litle endian processor
  memcpy(buffer, &calData.offset.accel, 6);
  memcpy(buffer + 6, &calData.offset.mag, 6);
  memcpy(buffer + 12, &calData.offset.gyro, 6);
  memcpy(buffer + 18, &calData.radius.accel, 2);
  memcpy(buffer + 20, &calData.radius.mag, 2);

  for (uint8_t i=0; i < 22; i++) {
    // TODO(oliv4945): create multibytes write
    bno055_writeData(BNO055_ACC_OFFSET_X_LSB+i, buffer[i]);
  }

  bno055_setOperationMode(operationMode);
}

bno055_vector_t bno055_getVector(uint8_t vec) {
  bno055_setPage(0);
  uint8_t buffer[8];    // Quaternion need 8 bytes

  if (vec == BNO055_VECTOR_QUATERNION)
    bno055_readData(vec, buffer, 8);
  else
    bno055_readData(vec, buffer, 6);

  double scale = 1;

  if (vec == BNO055_VECTOR_MAGNETOMETER) {
    scale = magScale;
  } else if (vec == BNO055_VECTOR_ACCELEROMETER ||
           vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
    scale = accelScale;
  } else if (vec == BNO055_VECTOR_GYROSCOPE) {
    scale = angularRateScale;
  } else if (vec == BNO055_VECTOR_EULER) {
    scale = eulerScale;
  } else if (vec == BNO055_VECTOR_QUATERNION) {
    scale = quaScale;
  }

  bno055_vector_t xyz = {.wg = 0, .xg = 0, .yg = 0, .zg = 0};
  if (vec == BNO055_VECTOR_QUATERNION) {
    xyz.wg = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.xg = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.yg = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    xyz.zg = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
  } else {
    xyz.xg = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.yg = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.zg = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
  }

  return xyz;
}

bno055_vector_t bno055_getVectorAccelerometer(void) {
  return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}
bno055_vector_t bno055_getVectorMagnetometer(void) {
  return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}
bno055_vector_t bno055_getVectorGyroscope(void) {
  return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}
bno055_vector_t bno055_getVectorEuler(void) {
  return bno055_getVector(BNO055_VECTOR_EULER);
}
bno055_vector_t bno055_getVectorLinearAccel(void) {
  return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}
bno055_vector_t bno055_getVectorGravity(void) {
  return bno055_getVector(BNO055_VECTOR_GRAVITY);
}
bno055_vector_t bno055_getVectorQuaternion(void) {
  return bno055_getVector(BNO055_VECTOR_QUATERNION);
}

void bno055_setAxisMap(bno055_axis_map_t axis) {
  uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
  uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
  bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
  bno055_writeData(BNO055_AXIS_MAP_SIGN, axisMapSign);
}
