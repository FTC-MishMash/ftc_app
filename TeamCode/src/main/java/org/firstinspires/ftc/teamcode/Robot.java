package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    //    public Servo linearLock;
    public Servo hanging;
    public ColorSensor colorRightFront;
    public ColorSensor colorLeftFront;
    public int shaftTargetPositionMarker = -300;
    float hsvValuesLeftFront[] = {0F, 0F, 0F};
    float hsvValuesRightFront[] = {0F, 0F, 0F};
    public final float valuesLeftFront[] = hsvValuesRightFront;
    public final float valuesRightFront[] = hsvValuesLeftFront;
    BNO055IMU imu;
    public double blueColorRightSensor = 100;
    public double redColorRightSensor = 42;
    public double blueColorLeftSensor = 135;
    public double redColorLeftSensor = 65;
    public double hangingOpenPosition = 0.2;
    public double hangingLockPosition = 0.7;
    public double angleTurnToImage = 261;
    public double newAngleTurnToImage = 70;
    public int SamplingAngleRight = 340;

    public int driveEncoderSamplingForward = 40;
    public int driveEncoderSamplingPositionSide = 30;
    public int driveEncoderSamplingPositionMiddle = 10;
    public int driveEncoderSamplingPositionSideBackward = -30;
    public int driveEncoderSamplingPositionMiddleBackward = -10;
    public int driveEncoderSamplingBackward = 0;
    public int SamplingAngleLeft = 20;
    public int distToDepot = 90;
    public int distToCrater = 200;
    public double powerEncoder = 0.7;
    int AngleToDepot = 135;
    int newAngleToDepot = 115;

    public Robot(HardwareMap hardwareMap) {

        inTake = hardwareMap.get(DcMotor.class, "inTake");

        hanging = hardwareMap.get(Servo.class, "hanging");
//        linearLock = hardwareMap.get(Servo.class, "LinearServoLock");

        linear = hardwareMap.get(DcMotor.class, "linearLeft");
        shaft[0] = hardwareMap.get(DcMotor.class, "shaftRight");
        shaft[1] = hardwareMap.get(DcMotor.class, "shaftLeft");

        colorRightFront = hardwareMap.get(ColorSensor.class, "colorRightFront");
        colorLeftFront = hardwareMap.get(ColorSensor.class, "colorLeftFront");

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

        shaft[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shaft[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
