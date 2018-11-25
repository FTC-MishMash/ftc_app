package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by user on 22/11/2018.
 */
@Disabled
public class getDown extends LinearOpMode {
    Robot robot;
    BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        DcMotor[][] motors;
        ElapsedTime runTime = new ElapsedTime();

    }

//    public void Down(double timeToStop) {
//        while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle > 0 && getRuntime() < timeToStop) {
//
//            robot.shaft[0].setPower(0.3);
//            robot.shaft[1].setPower(0.3);
//        }
//        robot.shaft[0].setPower(0);
//        robot.shaft[1].setPower(0);
//    }
}


