package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by user on 06/11/2018.
 */

public class Robot {
    public static String drive;
    public DcMotor[][] driveTrain;
    public DcMotor[] shaft = new DcMotor[2];
    public DcMotor linear;
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
//        shaft = new DcMotor[2];
//        linear = new DcMotor[2];
        inTake = hardwareMap.get(DcMotor.class, "inTake");

        linear = hardwareMap.get(DcMotor.class, "linearLeft");
        shaft[0] = hardwareMap.get(DcMotor.class, "shaftRight");
        shaft[1] = hardwareMap.get(DcMotor.class, "shaftLeft");
//
//        colorRightFront = hardwareMap.get(ColorSensor.class, "colorRightFront");
//        colorLeftFront = hardwareMap.get(ColorSensor.class, "colorLeftFront");

        driveTrain[0][0] = hardwareMap.get(DcMotor.class, "leftFront");
        driveTrain[1][0] = hardwareMap.get(DcMotor.class, "leftBack");
        driveTrain[0][1] = hardwareMap.get(DcMotor.class, "rightFront");
        driveTrain[1][1] = hardwareMap.get(DcMotor.class, "rightBack");

        shaft[0].setDirection(DcMotorSimple.Direction.FORWARD);
        shaft[1].setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrain[0][0].setDirection(DcMotorSimple.Direction.FORWARD);
        driveTrain[1][0].setDirection(DcMotorSimple.Direction.FORWARD);
        driveTrain[0][1].setDirection(DcMotorSimple.Direction.REVERSE);
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


//        shaft[0].setTargetPosition(0);
//        shaft[1].setTargetPosition(0);
//        linear.setTargetPosition(0);
//
//        shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
