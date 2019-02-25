package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.ClassFactoryImpl;
import org.firstinspires.ftc.robotcore.internal.tfod.TFObjectDetectorImpl;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import android.graphics.Color;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


/**
 * Created by user on 22/11/2018.
 */
@Autonomous(name = "AutoMode")
@Disabled
public class AutoMode extends LinearOpMode {
    /**
     * A master autonomous class that initializes all of the objects that
     * in the autonomous program and execute the initialization part before the match start/.
     */

    public Robot robot;   //An instance Robot class which provide access to the robot components.


    public ElapsedTime runTime = new ElapsedTime();
    final double SCALE_FACTOR = 255;

    /**
     * These two variables are the range values of the IMU pitch angle to tell
     * if robot is balanced on the field.
     */
    static final int PitchtargetAngleMin = -5;
    static final int PitchtargetAngleMax = 5;
    public VuforiaLocalizerEx vuforia;


    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod; //
    static final int RolltargetAngleMin = -10;
    static final int RolltargetAngleMax = 10;


    public static final float mmPerInch = 25.4f;
    public static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;//TODO
    ElapsedTime runtime = new ElapsedTime();
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    List<VuforiaTrackable> allTrackablesNav;
    ImageTargets targetNav;
    DriveUtilities driveUtils;
    TensorflowUtils tsSampling;
    TensorflowUtils.GOLD_MINERAL_POSITION goldPos = TensorflowUtils.GOLD_MINERAL_POSITION.NONE;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    @Override

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        targetNav = new ImageTargets(this);
        driveUtils = new DriveUtilities(this);
        tsSampling = new TensorflowUtils(this);
        tsSampling.initVuforia(true);
        robot.shaft[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


//    public boolean magneticShafts() {
//        return !robot.magnetShaftOpen.getState();
//    }

    public Orientation getAngularOriention() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void LandInAuto(double servoOPENPosition, double shaftPower) {
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition());
        robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition());
        robot.shaft[0].setPower(1);
        robot.shaft[1].setPower(1);
        sleep(75);
        robot.hanging.setPosition(robot.hangingLockPosition);

        robot.linear.setTargetPosition(-125);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setPower(0.7);

//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("pitch", getAngularOriention().thirdAngle);
        telemetry.addData("shaft[0] encoder", robot.shaft[0].getCurrentPosition());
        telemetry.addData("shaft[1] encoder", robot.shaft[1].getCurrentPosition());
        telemetry.update();
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sleep(150);
//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition() );
//        robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition() );
//        robot.shaft[0].setPower(1);
//        robot.shaft[1].setPower(1);
//        sleep(200);
//        robot.shaft[0].setPower(-1);
//        robot.shaft[1].setPower(-1);
        while (opModeIsActive() && getAngularOriention().thirdAngle <= 0) {
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition() - 250);
            robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition() - 250);
            robot.shaft[0].setPower(shaftPower);
            robot.shaft[1].setPower(shaftPower);

//            if (!magneticShafts()) {
//                robot.shaft[0].setPower(0);
//                robot.shaft[1].setPower(0);
//            }

//            sleep(60);
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition() + 10);
            robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition() + 10);
            robot.shaft[0].setPower(shaftPower);
            robot.shaft[1].setPower(shaftPower);

//            if (!magneticShafts()) {
//                robot.shaft[0].setPower(0);
//                robot.shaft[1].setPower(0);
//            }
            telemetry.addData("pitch", getAngularOriention().thirdAngle);
            telemetry.addData("shaft[0] encoder", robot.shaft[0].getCurrentPosition());
            telemetry.addData("shaft[1] encoder", robot.shaft[1].getCurrentPosition());

            telemetry.update();
        }
        robot.hanging.setPosition(robot.hangingOpenPosition);
        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);
        //TODO: add this
//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[0].setTargetPosition(-2700);
//        robot.shaft[1].setTargetPosition(-2700);
//        robot.shaft[0].setPower(1);
//        robot.shaft[1].setPower(1);
//
//        while (opModeIsActive() && robot.shaft[0].isBusy() && robot.shaft[1].isBusy()) {
//            telemetry.addData("pitch", getAngularOriention().thirdAngle);
//            telemetry.addData("shaft[0] encoder", robot.shaft[0].getCurrentPosition());
//            telemetry.addData("shaft[1] encoder", robot.shaft[1].getCurrentPosition());
//
//            telemetry.update();
//        }


//        robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition());
//        robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition());
//
//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.shaft[0].setPower(1);
//        robot.shaft[1].setPower(1);
//
//        robot.linear.setTargetPosition(-750);
//        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.linear.setPower(0.6);
//        sleep(300);
//
//        robot.linear.setTargetPosition(robot.linear.getCurrentPosition() + 300);
//        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.linear.setPower(1);
//        sleep(250);
//
//        robot.linear.setTargetPosition(-750);
//        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.linear.setPower(1);
//        double t1 = getRuntime();
//        while (opModeIsActive() && robot.linear.getCurrentPosition() >= -725 && getRuntime() - t1 <= 0.4) {
//            telemetry.addData("pitch", getAngularOriention().thirdAngle);
//            telemetry.addData("shaft[0] encoder", robot.shaft[0].getCurrentPosition());
//            telemetry.addData("shaft[1] encoder", robot.shaft[1].getCurrentPosition());
//            telemetry.addData("linear encoder", robot.linear.getCurrentPosition());
//            telemetry.update();
//        }
//
//        robot.shaft[0].setPower(0);
//        robot.shaft[1].setPower(0);
//
//        robot.linear.setPower(0);
//        robot.shaft[0].setPower(0);
//        robot.shaft[1].setPower(0);


    }


    public void shaftGoDown(double shaftPower, int shaftDownPosition) {
        DriveUtilities.setMotorPower(robot.driveTrain, new double[][]{{0.3, 0.3}, {0.3, 0.3}});

        sleep(300);
        DriveUtilities.setMotorPower(robot.driveTrain, new double[][]{{0, 0}, {0, 0}});


        robot.shaft[0].setTargetPosition(shaftDownPosition);
        robot.shaft[1].setTargetPosition(shaftDownPosition);

        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if (!magneticShafts()) {
//            robot.shaft[0].setPower(0);
//            robot.shaft[1].setPower(0);
//        }
//        sleep(400);
        /**
         * here the robot fold his arms to its original shape
         */
        robot.shaft[0].setPower(shaftPower);
        robot.shaft[1].setPower(shaftPower);

//        if (!magneticShafts()) {
//            robot.shaft[0].setPower(0);
//            robot.shaft[1].setPower(0);
//        }
        double t0 = getRuntime();
        while (opModeIsActive() &&
                robot.shaft[0].isBusy() &&
                robot.shaft[1].isBusy() &&
                getRuntime() - t0 < 1.5) {
            telemetry.addData("shaft go down", robot.shaft[0].isBusy());
            telemetry.update();
        }
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setTargetPosition(0);
        robot.linear.setPower(0.4);


        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);

        robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public double[] GyroPID(double heading, double lasterror, BNO055IMU imu) {
        double kp = 0.015, kd = 0.01, ki = 0, nexterror = 0;
        double err = heading - imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (err > 180)
            err = err - 360;
        while (err < -180)
            err = err + 360;
        lasterror = err - lasterror;
        double pd = nexterror * ki + lasterror * kd + err * kp;
        return (new double[]{-pd, err});
    }

    public void getOffTheClimb(BNO055IMU imu, DcMotor[] motorsHanging, double power) {
        setMotorSHAFTPower(power);
        while (!straightToField(imu)) ;
        setMotorSHAFTPower(0);
    }

    public boolean straightToField(BNO055IMU imu) {
        Orientation axis = DriveUtilities.getAngularOriention(imu);
        return axis.secondAngle > RolltargetAngleMin && axis.secondAngle < RolltargetAngleMax && axis.thirdAngle > PitchtargetAngleMin && axis.thirdAngle < PitchtargetAngleMax;

    }


    public void setMotorSHAFTPower(double power) { //Stores the four drivetrain motors power in array
        for (int row = 0; opModeIsActive() && row < 2; row++)
            robot.shaft[row].setPower(power);
    }

    public void MarkerWithIntake(double intakePower, int sleepTime) {
        robot.inTake.setPower(intakePower);
        sleep(sleepTime);
        robot.inTake.setPower(0);
    }

    public void Marker(double powerShaft, int shaftTargetPositionMarker) {
//        driveByColor(color, sensorcColor, imu, hsvValue, heading, power);
//        driveByEncoderRoverRuckus(75, 75, 0.5);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //open shaft
        robot.shaft[0].setTargetPosition(shaftTargetPositionMarker);
        robot.shaft[1].setTargetPosition(shaftTargetPositionMarker);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[0].setPower(powerShaft);
        robot.shaft[1].setPower(powerShaft);
//        if (!magneticShafts()) {
//            robot.shaft[0].setPower(0);
//            robot.shaft[1].setPower(0);
//        }
        while (opModeIsActive() &&
                robot.shaft[0].isBusy() &&
                robot.shaft[1].isBusy()) {
            telemetry.addData("shaft move  ", robot.shaft[0].isBusy());
            telemetry.update();
        }
        robot.inTake.setPower(1);
        sleep(800);
        robot.inTake.setPower(0);
        //marker
        robot.shaft[0].setTargetPosition(0);
        robot.shaft[1].setTargetPosition(0);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[0].setPower(powerShaft);
        robot.shaft[1].setPower(powerShaft);
        while (opModeIsActive() &&
                robot.shaft[0].isBusy() &&
                robot.shaft[1].isBusy()) {
            telemetry.addData("shaft move 2 ", robot.shaft[0].isBusy());
            telemetry.update();
        }
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);
//        if (!magneticShafts()) {
//            robot.shaft[0].setPower(0);
//            robot.shaft[1].setPower(0);
//        }
        robot.linear.setTargetPosition(0);
        robot.linear.setPower(0.4);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void Parking(int targetShaftParkingPositionEncoder, double shaftPower, int linearTargetEncoder,int LinearoutFromLock, double linearPower) {
        robot.linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveUtils.driveByEncoderRoverRuckus(160, 160, 0.5, false);
        robot.linear.setTargetPosition(LinearoutFromLock);
        robot.linear.setPower(linearPower);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && robot.linear.isBusy()) ;
        robot.linear.setPower(0);


        robot.shaft[0].setTargetPosition(targetShaftParkingPositionEncoder);//250
        robot.shaft[1].setTargetPosition(targetShaftParkingPositionEncoder);

//        robot.shaft[0].setTargetPosition(-750);
//        robot.shaft[1].setTargetPosition(-750);
        robot.shaft[0].setPower(shaftPower);
        robot.shaft[1].setPower(shaftPower);
        while (opModeIsActive() && robot.shaft[0].isBusy() && robot.shaft[1].isBusy()) ;
        robot.linear.setTargetPosition(linearTargetEncoder);
        robot.linear.setPower(linearPower);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && robot.linear.isBusy()) ;
        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);
        robot.linear.setPower(0);
        robot.linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }


}