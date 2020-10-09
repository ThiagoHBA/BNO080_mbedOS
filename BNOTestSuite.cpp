/* 
	USC RPL HAMSTER v2.3
	Contributors: Lauren Potterat 
*/

#include "BNOTestSuite.h"
#include <sstream>
#include <cinttypes>
#include <iostream>
#include <chrono>
#include "mbed.h"
#include "rtos.h"

#define RAD_TO_DEG (180.0/M_PI)

using namespace std;
using namespace chrono;


ostringstream ss;
Timer tempo_millis;


void BNOTestSuite::test_printInfo()
{
	pc.printf("BNO080 reports as SW version %" PRIu8 ".%" PRIu8 ".%" PRIu16", build %" PRIu32 ", part no. %" PRIu32"\n",
			imu.majorSoftwareVersion, imu.minorSoftwareVersion, imu.patchSoftwareVersion,
			imu.buildNumber, imu.partNumber);
}

void BNOTestSuite::test_readRotationVector() 
{
	const size_t update_rate_ms = 200;

	imu.enableReport(BNO080::ROTATION, update_rate_ms);

	while (true)
	{
		ThisThread::sleep_for(1ms * update_rate_ms);

        if(imu.updateData())
		{
        	TVector3 eulerRadians = imu.rotationVector.euler();
        	TVector3 eulerDegrees = eulerRadians * RAD_TO_DEG;
            

			pc.printf("IMU Rotation Euler: %f || Roll: %f deg, Pitch: %f deg, Yaw: %f deg \r\n", (float)5.55,
					   (float)eulerDegrees[0],(float)eulerDegrees[1], (float)eulerDegrees[2]);

        	pc.printf(", Accuracy: %.02f", (imu.rotationAccuracy * RAD_TO_DEG));

        	pc.printf(", Status: %s\n", imu.getReportStatusString(BNO080::ROTATION));
		}
		else
		{
			pc.printf("IMU was not ready with data packet!\r\n");
		}
    }
}

void BNOTestSuite::test_readRotationAcceleration() // SPI_PRINT
{
	imu.enableReport(BNO080::ROTATION, 10);
	imu.enableReport(BNO080::LINEAR_ACCELERATION, 10);
    
    

	while (true)
	{
        tempo_millis.start();

		ThisThread::sleep_for(1ms);

		if(imu.updateData())
		{  
            
			TVector3 eulerRadians = imu.rotationVector.euler();
			TVector3 eulerDegrees = eulerRadians * (180.0 / M_PI);
            
            printf("%f : %f : %f : ",imu.linearAcceleration[0],imu.linearAcceleration[1],imu.linearAcceleration[2]);
            printf("%f : %f : %f : ",eulerDegrees[0],eulerDegrees[1],eulerDegrees[2]);
            tempo_millis.stop();
            printf("%llu\n", duration_cast<milliseconds>(tempo_millis.elapsed_time()).count());
		}

        
        
	}
}

void BNOTestSuite::test_readLinearAcceleration()
{
	imu.enableReport(BNO080::LINEAR_ACCELERATION, 10);

	while (true)
	{
		ThisThread::sleep_for(1ms);

		if(imu.updateData())
		{
			if (imu.hasNewData(BNO080::LINEAR_ACCELERATION))
			{
				pc.printf("IMU Linear Acceleration: ");
				imu.linearAcceleration.print(pc, true);
				pc.printf("\n");
			}
		}
	}
}

void BNOTestSuite::test_tapDetector()
{
	imu.enableReport(BNO080::TAP_DETECTOR, 10);
	pc.printf("Listening for taps...\n");

	while(true)
	{
		if(imu.updateData())
		{
			if(imu.tapDetected)
			{
				pc.printf("Tap detected!\n");
				imu.tapDetected = false;
			}
		}
	}
}

void BNOTestSuite::test_gameRotationVector()
{
	const size_t update_rate_ms = 200;

	imu.enableReport(BNO080::GAME_ROTATION, update_rate_ms);

	while (true)
	{
		ThisThread::sleep_for(1ms * update_rate_ms);

		if(imu.updateData())
		{
			pc.printf("IMU Game Rotation Euler: [");
			TVector3 eulerRadians = imu.gameRotationVector.euler();
			TVector3 eulerDegrees = eulerRadians * RAD_TO_DEG;
			eulerDegrees.print(pc, true);

			pc.printf("], Status: %s\n", imu.getReportStatusString(BNO080::GAME_ROTATION));
		}
		else
		{
			pc.printf("IMU was not ready with data packet!\r\n");
		}
	}
}

void BNOTestSuite::test_tare()
{
	const size_t update_rate_ms = 100;
	imu.enableReport(BNO080::ROTATION, update_rate_ms);

	Timer runningTime;
	runningTime.start();

	bool sentTareCmd = false;

	while (true)
	{
		ThisThread::sleep_for(1ms * update_rate_ms);

		if(imu.updateData())
		{
			pc.printf("IMU Rotation Euler: [");
			TVector3 eulerRadians = imu.rotationVector.euler();
			TVector3 eulerDegrees = eulerRadians * RAD_TO_DEG;
			eulerDegrees.print(pc, true);

			pc.printf("], Status: %s\n", imu.getReportStatusString(BNO080::ROTATION));
		}
		else
		{
			pc.printf("IMU was not ready with data packet!\r\n");
		}

		if(runningTime.read() > 2 && !sentTareCmd)
		{
			pc.printf("2 seconds have passed, sending tare command...\n");
			imu.tare();
			sentTareCmd = true;
		}
	}
}

void BNOTestSuite::test_magCalibration()
{
	// enable calibration for the magnetometer
	bool success = imu.enableCalibration(false, false, true);

	const size_t update_rate_ms = 200;
	imu.enableReport(BNO080::GAME_ROTATION, update_rate_ms);
	imu.enableReport(BNO080::MAG_FIELD, update_rate_ms);
	imu.enableReport(BNO080::MAG_FIELD_UNCALIBRATED, update_rate_ms);

	if(success)
	{
		pc.printf("Calibration mode was successfully enabled!\n");
	}
	else
	{
		pc.printf("Calibration mode failed to enable!\n");
		while(true){}
	}

	while (true)
	{
		ThisThread::sleep_for(1ms * update_rate_ms);

		if(imu.updateData())
		{
			pc.printf("IMU Magnetic Field: [");
			imu.magFieldUncalibrated.print(pc, true);

			pc.printf("] uTesla, Status: %hhu (should be 2-3 when calibration is complete), ", imu.getReportStatus(BNO080::MAG_FIELD));

			pc.printf("Hard iron offsets: [");
			imu.hardIronOffset.print(pc, true);

			pc.printf("]\n");

		}
		else
		{
			pc.printf("IMU was not ready with data packet!\r\n");
		}
	}

}

void BNOTestSuite::test_stabilityClassifier()
{
	imu.enableReport(BNO080::STABILITY_CLASSIFIER, 200);

	while (true)
	{
        ThisThread::sleep_for(1ms);

		if(imu.updateData())
		{
			if (imu.hasNewData(BNO080::STABILITY_CLASSIFIER))
			{
				pc.printf("Stability: ");

				switch(imu.stability)
				{
					case BNO080::UNKNOWN:
						pc.printf("Unknown\n");
						break;
					case BNO080::ON_TABLE:
						pc.printf("On Table\n");
						break;
					case BNO080::STATIONARY:
						pc.printf("Stationary\n");
						break;
					case BNO080::STABLE:
						pc.printf("Stable\n");
						break;
					case BNO080::MOTION:
						pc.printf("Motion\n");
						break;
				}
			}
		}
	}

}

void BNOTestSuite::test_metadata()
{
	pc.printf("Printing metadata for Linear Acceleration:\r\n");
	imu.printMetadataSummary(BNO080::LINEAR_ACCELERATION);

	pc.printf("Printing metadata for Rotation Vector:\r\n");
	imu.printMetadataSummary(BNO080::ROTATION);

	pc.printf("Printing metadata for Magnetic Field:\r\n");
	imu.printMetadataSummary(BNO080::MAG_FIELD);

	pc.printf("Printing metadata for Tap Detector:\r\n");
	imu.printMetadataSummary(BNO080::TAP_DETECTOR);

}

void BNOTestSuite::test_orientation()
{
	const size_t update_rate_ms = 200;
	imu.enableReport(BNO080::TOTAL_ACCELERATION, update_rate_ms);
	imu.enableReport(BNO080::ROTATION, update_rate_ms);

	Timer runningTime;
	runningTime.start();

	bool setOrientation = false;

	while (true)
	{
		ThisThread::sleep_for(1ms * update_rate_ms);

		if(imu.updateData())
		{
			if (imu.hasNewData(BNO080::ROTATION))
			{
				pc.printf("IMU Rotation Euler: ");
				TVector3 eulerRadians = imu.rotationVector.euler();
				TVector3 eulerDegrees = eulerRadians * (180.0 / M_PI);
				eulerDegrees.print(pc, true);
				pc.printf("\n");
			}
			if (imu.hasNewData(BNO080::TOTAL_ACCELERATION))
			{
				pc.printf("IMU Total Acceleration: ");
				imu.totalAcceleration.print(pc, true);
				pc.printf("\n");
			}
		}

		if(runningTime.read() > 2 && !setOrientation)
		{
			pc.printf("2 seconds have passed, sending set orientation command...\n");
			setOrientation = true;

			// rotate sensor about the Y axis, so Z points down and X points west.
			// (values from BNO datasheet page 41)
			imu.setSensorOrientation(Quaternion(0, -1, 0, 0));
		}
	}
}

void BNOTestSuite::test_permanentOrientation()
{
	pc.printf("Setting permanent sensor orientation...\n");
	Timer orientationTimer;
	orientationTimer.start();

	// rotate sensor 90 degrees CCW about the Y axis, so X points up, Y points back, and Z points out
	// (values from BNO datasheet page 41)
	// imu.setPermanentOrientation(Quaternion(1.0f/SQRT_2, -1.0f/SQRT_2, 0, 0));

	// rotate sensor 180 degrees about Y axis
	//with these quaternion values x axis is oriented to point toward BRB, Z points up and Y points out towards you when you face the unit
	// see page 5 and 11 of hillcrestlabs FSM30X datasheet
	imu.setPermanentOrientation(Quaternion(0,-1, 0, 0)); 
	
	// reset to default orientation
	//imu.setPermanentOrientation(Quaternion(0, 0, 0, 0));

	orientationTimer.stop();
	pc.printf("Done setting orientation (took %.03f seconds)\r\n", orientationTimer.read());

}

void BNOTestSuite::test_disable()
{
	const size_t update_rate_ms = 200;
	imu.enableReport(BNO080::TOTAL_ACCELERATION, update_rate_ms);
	imu.enableReport(BNO080::ROTATION, update_rate_ms);

	Timer runningTime;
	runningTime.start();

	bool disabledRotation = false;

	while (true)
	{
		ThisThread::sleep_for(1ms * update_rate_ms);

		if(imu.updateData())
		{
			if (imu.hasNewData(BNO080::ROTATION))
			{
				pc.printf("IMU Rotation Euler: ");
				TVector3 eulerRadians = imu.rotationVector.euler();
				TVector3 eulerDegrees = eulerRadians * (180.0 / M_PI);
				eulerDegrees.print(pc, true);
				pc.printf("\n");
			}
			if (imu.hasNewData(BNO080::TOTAL_ACCELERATION))
			{
				pc.printf("IMU Total Acceleration: ");
				imu.totalAcceleration.print(pc, true);
				pc.printf("\n");
			}
		}

		if(runningTime.read() > 2 && !disabledRotation)
		{
			pc.printf("2 seconds have passed, disabling rotation report...\n");
			disabledRotation = true;

			imu.disableReport(BNO080::ROTATION);
		}
	}
}

void BNOTestSuite::test_accelCalibration()
{
	// enable calibration for the magnetometer
	bool success = imu.enableCalibration(true, false, false);

	const size_t update_rate_ms = 200;
	imu.enableReport(BNO080::Report::TOTAL_ACCELERATION, update_rate_ms);

	if(success)
	{
		pc.printf("Calibration mode was successfully enabled!\r\n");
	}
	else
	{
		pc.printf("Calibration mode failed to enable!\r\n");
		while(true){}
	}

	while (true)
	{
		ThisThread::sleep_for(1ms * update_rate_ms);

		if(imu.updateData())
		{
			pc.printf("IMU Total Acceleration: [");
			imu.totalAcceleration.print(pc, true);
			pc.printf("] m/s^2, Status: %hhu (should be 2-3 when calibration is complete), ", imu.getReportStatus(BNO080::Report::TOTAL_ACCELERATION));
			pc.printf("]\r\n");

		}
		else
		{
			pc.printf("IMU was not ready with data packet!\r\n");
		}

		if(serial.readable())
		{
			// char typed over serial
			pc.printf("Saving current calibration...\r\n");
			imu.saveCalibration();

			// throw out rest of chars in serial buffer
			while(serial.readable())
			{
				pc.getc();
			}
		}
	}

}

int main()
{
	pc.printf("============================================================\n");

	// reset and connect to IMU
	while(!imu.begin())
	{
		pc.printf("Failed to connect to IMU!\r\n");
        ThisThread::sleep_for(500);
	}

	//Declare test harness
	BNOTestSuite harness;

	int test = -1;
	pc.printf("\r\n\nHamster BNO Test Suite:\r\n");

	//MENU. ADD OPTION PER EACH TEST
	pc.printf("Select a test: \n\r");
	pc.printf("1. Print linearAcceleration\r\n");
	pc.printf("2. Display rotation vector\r\n");
	pc.printf("3. Display rotation and acceleration |Envio para raspberry| \r\n");
	pc.printf("4. Run tap detector\r\n");
	pc.printf("5. Display game rotation vector\r\n");
	pc.printf("6. Test tare function\r\n");
	pc.printf("7. Run magnetometer calibration\r\n");
	pc.printf("8. Test stability classifier\r\n");
	pc.printf("9. Test metadata reading (requires BNO_DEBUG = 1)\r\n");
	pc.printf("10. Test set orientation\r\n");
	pc.printf("11. Test set permanent orientation\r\n");
	pc.printf("12. Test disabling reports\r\n");
	pc.printf("13. Test accelerometer calibration\r\n");

	pc.scanf("%d", &test);
	pc.printf("Running test %d:\r\n\n", test);


	//SWITCH. ADD CASE PER EACH TEST
	switch(test){
        case 1:
            harness.test_readLinearAcceleration();
		case 2:
			harness.test_readRotationVector();
			break;
		case 3:
			harness.test_readRotationAcceleration();
			break;
		case 4:
			harness.test_tapDetector();
			break;
		case 5:
			harness.test_gameRotationVector();
			break;
		case 6:
			harness.test_tare();
			break;
		case 7:
			harness.test_magCalibration();
			break;
		case 8:
			harness.test_stabilityClassifier();
			break;
		case 9:
			harness.test_metadata();
			break;
		case 10:
			harness.test_orientation();
			break;
		case 11:
			harness.test_permanentOrientation();
			break;
		case 12:
			harness.test_disable();
			break;
		default:
			printf("Invalid test number. Please run again.\r\n");
			return 1;
	}

	pc.printf("Done!\r\n");
	return 0;

}









