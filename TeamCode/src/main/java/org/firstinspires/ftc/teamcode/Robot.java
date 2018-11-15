package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by user on 06/11/2018.
 */

public class Robot {
    public DcMotor[][] driveTrain;
    BNO055IMU imu;

    public Robot(HardwareMap hardwareMap) {
        driveTrain = new DcMotor[2][2];
        driveTrain[0][0] = hardwareMap.get(DcMotor.class, "leftFront");
        driveTrain[1][0] = hardwareMap.get(DcMotor.class, "leftBack");
        driveTrain[0][1] = hardwareMap.get(DcMotor.class, "rightFront");
        driveTrain[1][1] = hardwareMap.get(DcMotor.class, "rightBack");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public DcMotor[][] getDriveTrain() {
        return driveTrain;
    }
}
