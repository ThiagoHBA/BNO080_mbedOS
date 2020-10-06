/* 
	USC RPL HAMSTER v2.3
	Contributors: Lauren Potterat 
*/

#pragma once

#include <mbed.h>

#include "BNO080.h"
#include "SerialStream.h"

BufferedSerial serial(USBTX, USBRX, 115200);
SerialStream<BufferedSerial> pc(serial);

// These pin assignments are specific to my dev setup -- you'll need to change them
BNO080 imu(&pc, PinName(PB_9), PinName(PB_8), PinName(D8), PinName(D4), 0x4B, 100000); 

class BNOTestSuite{
public:

	void test_printInfo();

	void test_readRotationVector();

	void test_readRotationAcceleration();

	void test_tapDetector();

	void test_gameRotationVector();

	void test_tare();

	void test_magCalibration();

	void test_accelCalibration();

	void test_stabilityClassifier();

	void test_metadata();

	void test_orientation();
	
	void test_permanentOrientation();

	void test_disable();

    void test_readLinearAcceleration();

};
