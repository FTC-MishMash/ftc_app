package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

/**
 * Created by user on 06/11/2018.
 */

public class Robot {
    public DcMotor[][] driveTrain;
    public DcMotor[] shaft;
    public DcMotor[] linear;
    public DcMotor inTake;
    public ColorSensor colorRightFront;
    public ColorSensor colorLeftFront;
    float hsvValuesLeftFront[] = {0F, 0F, 0F};
    float hsvValuesRightFront[] = {0F, 0F, 0F};
    public final float valuesLeftFront[] = hsvValuesRightFront;
    public final float valuesRightFront[] = hsvValuesLeftFront;
    BNO055IMU imu;
    public double blueColorRightSensor = 100;
    public double redColorRightSensor = 42;
    public double blueColorLeftSensor = 135;
    public double redColorLeftSensor = 65;


    public Robot(HardwareMap hardwareMap) {
        driveTrain = new DcMotor[2][2];
        shaft = new DcMotor[2];
        linear = new DcMotor[2];
//        inTake = hardwareMap.get(DcMotor.class, "inTake");
//
//        linear[0] = hardwareMap.get(DcMotor.class, "linearRight");
//        linear[1] = hardwareMap.get(DcMotor.class, "linearLeft");
//
//        colorRightFront = hardwareMap.get(ColorSensor.class, "colorRightFront");
//        colorLeftFront = hardwareMap.get(ColorSensor.class, "colorLeftFront");
        driveTrain[0][0] = hardwareMap.get(DcMotor.class, "leftFront");
        driveTrain[1][0] = hardwareMap.get(DcMotor.class, "leftBack");
        driveTrain[0][1] = hardwareMap.get(DcMotor.class, "rightFront");
        driveTrain[1][1] = hardwareMap.get(DcMotor.class, "rightBack");

        driveTrain[0][0].setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain[0][1].setDirection(DcMotorSimple.Direction.FORWARD);
        driveTrain[1][0].setDirection(DcMotorSimple.Direction.FORWARD);
        driveTrain[1][1].setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrain[0][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain[0][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain[1][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain[1][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        shaft[0] = hardwareMap.get(DcMotor.class, "shaft0");
//        shaft[1] = hardwareMap.get(DcMotor.class, "shaft1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public BNO055IMU getImu() {
        return imu;
    }

    public DcMotor[] getShaft() {
        return shaft;
    }

    public DcMotor[][] getDriveTrain() {
        return driveTrain;
    }
}
