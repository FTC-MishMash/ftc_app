package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by user on 06/11/2018.
 */

public class Robot {

    public DcMotor[][] driveTrain = new DcMotor[2][2];
    public DcMotor[] shaft = new DcMotor[2];
    public DcMotor linear;
    public DcMotor inTake;
    public Servo hanging;
    public Servo mineralHolder;

    public DigitalChannel magnetShaftOpen;

    public int driveEncoderAfHanging = 9;


    BNO055IMU imu;
    public double cubesTriggerOPENposition = 0.8;
    public double cubesTriggerCLOSEposition = 0.3;
    public double hangingOpenPosition = 0.7;
    public double hangingLockPosition = 0.2;
    public double angleTurnToImage = 264;
    public double newAngleTurnToImage = 70;
    public int SamplingAngleRight = 326;

    public int driveEncoderSamplingForward = 31;
    public int driveEncoderSamplingPositionSide = 30;
    public int driveEncoderSamplingPositionMiddle = 15;
    public int driveEncoderSamplingPositionSideBackward = -30;
    public int driveEncoderSamplingPositionMiddleBackward = -18;
    public int driveEncoderSamplingBackward = -7;
    public int SamplingAngleLeft = 33;
    public int distToDepot = 90;
    public int distToImageBeforeCrater = -70;
    public int distFromImageToCrater = 35;
    public double powerEncoder = 0.8;

    public int encoderBACKUPtoImage = -67;

    public int linearOpenPosition = -2000;
    public int linearMiddlePosition = -800;
    public int linearClosePosition = -50;
    public int linearEncoderOutLock = -300;

    public int shaftEncoderPositionPARKING = -800;
    public int shaftEncoderPositionINTAKE = -250;
    public int shaftEncoderPosition90deg = -2400;
    public int shaftTargetPositionMarker = -300;
    public int shaftDownPosition = 100;
    //    int AngleToDepot = 135;
    public int newAngleToDepot = 195;


    public Robot(HardwareMap hardwareMap) {

        inTake = hardwareMap.get(DcMotor.class, "inTake");


        linear = hardwareMap.get(DcMotor.class, "linearLeft");
        shaft[0] = hardwareMap.get(DcMotor.class, "shaftRight");
        shaft[1] = hardwareMap.get(DcMotor.class, "shaftLeft");


        magnetShaftOpen = hardwareMap.get(DigitalChannel.class, "magnetShaftOpen");
        hanging = hardwareMap.get(Servo.class, "hanging");
        mineralHolder = hardwareMap.get(Servo.class, "mineralHolder");

        driveTrain[0][0] = hardwareMap.get(DcMotor.class, "leftFront");
        driveTrain[1][0] = hardwareMap.get(DcMotor.class, "leftBack");
        driveTrain[0][1] = hardwareMap.get(DcMotor.class, "rightFront");
        driveTrain[1][1] = hardwareMap.get(DcMotor.class, "rightBack");

        linear.setDirection(DcMotorSimple.Direction.FORWARD);
        inTake.setDirection(DcMotorSimple.Direction.FORWARD);
        shaft[0].setDirection(DcMotorSimple.Direction.REVERSE);
        shaft[1].setDirection(DcMotorSimple.Direction.FORWARD);
//
        driveTrain[0][0].setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain[1][0].setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain[0][1].setDirection(DcMotorSimple.Direction.FORWARD);
        driveTrain[1][1].setDirection(DcMotorSimple.Direction.FORWARD);

        driveTrain[0][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain[0][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain[1][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain[1][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        shaft[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shaft[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shaft[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shaft[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        inTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
}
