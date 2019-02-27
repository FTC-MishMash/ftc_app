package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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
    TensorflowUtils.MINERAL_POSITION goldPos = TensorflowUtils.MINERAL_POSITION.NONE;

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

    public void LandInAuto() {
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition());
        robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition());
        robot.shaft[0].setPower(1);
        robot.shaft[1].setPower(1);

        robot.hanging.setPosition(robot.hangingLockPosition);
        sleep(75);
        robot.linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setTargetPosition(robot.linearEncoderOutLock);
        robot.linear.setPower(robot.linearPowerOutLock);

//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("pitch", getAngularOriention().thirdAngle);
        telemetry.addData("shaft[0] encoder", robot.shaft[0].getCurrentPosition());
        telemetry.addData("shaft[1] encoder", robot.shaft[1].getCurrentPosition());
        telemetry.update();
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && getAngularOriention().thirdAngle <= -7) {

            robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition() - 400);
            robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition() - 400);
            robot.shaft[0].setPower(0.5);
            robot.shaft[1].setPower(0.5);
            while (robot.shaft[0].isBusy() && robot.shaft[1].isBusy()) ;
//            sleep(80);
            robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition() + 10);
            robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition() + 10);
            robot.shaft[0].setPower(1);
            robot.shaft[1].setPower(1);


            telemetry.addData("pitch", getAngularOriention().thirdAngle);
            telemetry.addData("shaft[0] encoder", robot.shaft[0].getCurrentPosition());
            telemetry.addData("shaft[1] encoder", robot.shaft[1].getCurrentPosition());

            telemetry.update();
        }
        robot.hanging.setPosition(robot.hangingOpenPosition);
        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);


    }


    public void shaftGoDown(double shaftPower, int shaftDownPosition) {

        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[0].setTargetPosition(0);
        robot.shaft[1].setTargetPosition(0);


//        sleep(400);

        robot.shaft[0].setPower(shaftPower);
        robot.shaft[1].setPower(shaftPower);


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
        robot.linear.setPower(1);


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


    public void MarkerWithIntake(double intakePower, int sleepTime) {
        robot.inTake.setPower(intakePower);
        sleep(sleepTime);
        robot.inTake.setPower(0);
    }

    //    public void Marker(double powerShaft, int shaftTargetPositionMarker) {
////        driveByColor(color, sensorcColor, imu, hsvValue, heading, power);
////        driveByEncoderRoverRuckus(75, 75, 0.5);
//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //open shaft
//        robot.shaft[0].setTargetPosition(shaftTargetPositionMarker);
//        robot.shaft[1].setTargetPosition(shaftTargetPositionMarker);
//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[0].setPower(powerShaft);
//        robot.shaft[1].setPower(powerShaft);
////        if (!magneticShafts()) {
////            robot.shaft[0].setPower(0);
////            robot.shaft[1].setPower(0);
////        }
//        while (opModeIsActive() &&
//                robot.shaft[0].isBusy() &&
//                robot.shaft[1].isBusy()) {
//            telemetry.addData("shaft move  ", robot.shaft[0].isBusy());
//            telemetry.update();
//        }
//        robot.inTake.setPower(1);
//        sleep(800);
//        robot.inTake.setPower(0);
//        //marker
//        robot.shaft[0].setTargetPosition(0);
//        robot.shaft[1].setTargetPosition(0);
//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[0].setPower(powerShaft);
//        robot.shaft[1].setPower(powerShaft);
//        while (opModeIsActive() &&
//                robot.shaft[0].isBusy() &&
//                robot.shaft[1].isBusy()) {
//            telemetry.addData("shaft move 2 ", robot.shaft[0].isBusy());
//            telemetry.update();
//        }
//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.shaft[0].setPower(0);
//        robot.shaft[1].setPower(0);
////        if (!magneticShafts()) {
////            robot.shaft[0].setPower(0);
////            robot.shaft[1].setPower(0);
////        }
//        robot.linear.setTargetPosition(0);
//        robot.linear.setPower(0.4);
//        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    }
    public void Sampling_secondTime(TensorflowUtils.MINERAL_POSITION goldPosition, int linearEncoderFIRST, int linearEncoderSecond,
                                    int linearEncoderThird, int shaft90degreesPosition, int linearEncoderMoveIntake
            , int LinearoutFromLock, double linearPower, double shaftPower) {
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setTargetPosition(LinearoutFromLock);
        robot.linear.setPower(linearPower);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && robot.linear.isBusy()) ;
        robot.linear.setPower(0);


        robot.shaft[0].setTargetPosition(shaft90degreesPosition);//250
        robot.shaft[1].setTargetPosition(shaft90degreesPosition);

        robot.shaft[0].setPower(shaftPower);
        robot.shaft[1].setPower(shaftPower);
        while (opModeIsActive() && robot.shaft[0].isBusy() && robot.shaft[1].isBusy()) ;



        if (goldPosition == TensorflowUtils.MINERAL_POSITION.LEFT) {
            robot.linear.setTargetPosition(linearEncoderFIRST);
        } else if (goldPosition == TensorflowUtils.MINERAL_POSITION.CENTER) {
            robot.linear.setTargetPosition(linearEncoderSecond);
        } else {
            robot.linear.setTargetPosition(linearEncoderThird);
        }
        robot.linear.setPower(linearPower);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && robot.linear.isBusy()) ;

    }


    public void Parking(int targetShaftParkingPositionEncoder, double shaftPower, int linearTargetEncoder, int LinearoutFromLock, double linearPower) {
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