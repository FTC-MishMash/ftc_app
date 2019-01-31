package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
//@Disabled
public class AutoMode extends LinearOpMode {
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    //    private static final android.graphics.Color Color = ;
    public Robot robot;
    public static final String VUFORIA_KEY = " ATgDONj/////AAABmW0G/nQirUMiumnzPc6Pl8oJhBOCC2qoUq0BWhir9YWcBFDlhZUfSwATcQArcyyLxIOV21sHaYJeeQEJZfIJ+4spBn3oJ/DfycsbPaNs87+TRpM46/vbUkj1Ok+NtZ/eqMhmMXjFC8dgdCfbCt0aMxoBNzDw4+v28abG+hjUCjVYf86Jq1m7R942XCjw0yhOZqTXWIp3WAZDXY/PdWGQGY/zWae0l6TAZ6Z27t1xYJdkkpLqEsbKM3ZprvtgIs8AsWS9Tri2892OHq2CnCL+1ZHHXKPdxON3fiC1Gd3oihwPhTUReNw0VAg9yeVsVa1UQg7ea9K6WpmVto0FG+T2/LV8uq/3Mp/NHWiNizw2DM4h";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    public ElapsedTime runTime = new ElapsedTime();
    final double SCALE_FACTOR = 255;
    static final int PitchtargetAngleMin = -5;
    static final int PitchtargetAngleMax = 5;
    public VuforiaLocalizerEx vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;
    static final int RolltargetAngleMin = -10;
    static final int RolltargetAngleMax = 10;

    //test1
    final double minAngleToTarget = 35;
    static final int XtargetPosition = 63;
    static final int YtargetPosition = 6;
    static final int ZtargetPosition = -4;

    static final int HeadingToSampling = 45;
    static final int HeadingToTarget = 90;

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
        tsSampling.initVuforiaWebCam(true);


    }


    public int searchCube(double power, int turnAngleRight, int turnAngleLeft, DcMotor[][] motor, BNO055IMU imu) {
        int cubePosition = 0;
        if (tfod == null) {
            telemetry.addData("tfod is NULL  ", tfod);
            telemetry.update();
            cubePosition = -1;
            return cubePosition;
        } else if (tfod != null) {
            tfod.activate();
        }

        double runTime0 = getRuntime();
        while (opModeIsActive() && getRuntime() - runTime0 < 3) {
            java.util.List<Recognition> RecognitionList = tfod.getUpdatedRecognitions();
            telemetry.addData("have cube?   ", RecognitionList != null);
            telemetry.update();

            if (RecognitionList != null)
                for (Recognition recognition : RecognitionList) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        cubePosition = 1;//CENTER
                        return cubePosition;
                    }
                }
        }
        //only if dont have cube in middle
        ScaledTurn(turnAngleRight, motor, imu, power);
        runTime0 = getRuntime();
        while (opModeIsActive() && getRuntime() - runTime0 < 2) {
            java.util.List<Recognition> RecognitionList = tfod.getUpdatedRecognitions();
            telemetry.addLine("have cube?   ");
            telemetry.update();

            if (RecognitionList != null)//TODO: להבין למה לא הלך שמאלה למרות שהייתה קוביה בימין
                for (Recognition recognition : RecognitionList) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        cubePosition = 2;//RIGHT
                        return cubePosition;
                    }
                }
        }

        // cubePosition != 1 && cubePosition != 2
        cubePosition = 3;//LEFT
        ScaledTurn(turnAngleLeft, motor, imu, power);
        runTime0 = getRuntime();
        while (opModeIsActive() && getRuntime() - runTime0 < 2) {
            List<Recognition> RecognitionList = tfod.getUpdatedRecognitions();
            telemetry.addLine("have cube?   ");
            telemetry.update();

            if (RecognitionList != null)//TODO: להבין למה לא הלך שמאלה למרות שהייתה קוביה בימין
                for (Recognition recognition : RecognitionList) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        cubePosition = 3;//RIGHT
                        return cubePosition;
                    }
                }
        }
        ScaledTurn(0, motor, imu, power);
        return cubePosition;
    }

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

        robot.linear.setTargetPosition(-125);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setPower(0.7);
        robot.hanging.setPosition(robot.hangingLockPosition);
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
            robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition() - 250);
            robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition() - 250);
            robot.shaft[0].setPower(shaftPower);
            robot.shaft[1].setPower(shaftPower);
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sleep(60);
            robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition() + 10);
            robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition() + 10);
            robot.shaft[0].setPower(-shaftPower);
            robot.shaft[1].setPower(-shaftPower);
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        setMotorPower(new double[][]{{0.3, 0.3}, {0.3, 0.3}});

        sleep(300);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});


        robot.shaft[0].setTargetPosition(shaftDownPosition);
        robot.shaft[1].setTargetPosition(shaftDownPosition);

        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        sleep(400);
        /**
         * here the robot fold his arms to its original shape
         */
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
robot.linear.setPower(0.4);
        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);

        robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveByColor(int color, ColorSensor sensorColor, BNO055IMU imu, float hsvValues[], double heading, double power)//0=red, blue=1
    {
        double redColorSensor = robot.redColorLeftSensor;
        double blueColorSensor = robot.blueColorLeftSensor;
        ResetHue(sensorColor, hsvValues);
        double pidErr[] = {0, 0};
        telemetry.addData("hsvValues[0]", hsvValues[0]);
        telemetry.update();
        if (color == 0 && opModeIsActive()) {
            double time = getRuntime();
            while (opModeIsActive() && hsvValues[0] > redColorSensor && (time + 2 > getRuntime())) {
                ResetHue(sensorColor, hsvValues);
                pidErr = GyroPID(heading, pidErr[1], imu);
                setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
                telemetry.addLine("ontheGrey");
                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("hsvValues[0]", hsvValues[0]);
                telemetry.update();

            }


        }

        if (color == 1 && opModeIsActive()) {
            double time1 = getRuntime();

            while (opModeIsActive() && hsvValues[0] < blueColorSensor && (time1 + 2 > getRuntime())) {
                ResetHue(sensorColor, hsvValues);
                pidErr = GyroPID(heading, pidErr[1], imu);
                setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
                telemetry.addLine("InDriveBlue");
                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("hsvValues[0]", hsvValues[0]);
                telemetry.update();

            }
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});


    }

    public void startTracking() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = CAMERA_CHOICE;
//        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        initVuforiaPhoneCamera();
//
//         Load the data sets that for the trackable objects. These particular data
//         sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackablesNav = new ArrayList<VuforiaTrackable>();
        allTrackablesNav.addAll(targetsRoverRuckus);


        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);


        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);


        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);


        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));
        //T
        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackablesNav) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, CAMERA_CHOICE);
        }

        /** Wait for the game to begin */


        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
    }

    public int getCube() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first..

        int cubePlace = -1;// 0 = NOT HERE, 1 = RIGHT (in camera), 2 = LEFT (in camera)

        if (tfod != null) {
            tfod.activate();
        }
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());


//            if (updatedRecognitions.size() == 1 && updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
//
//            }
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {//TODO: add cube place
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {

                            telemetry.addData("Gold Mineral Position", "Left");
                            cubePlace = 2;//Left
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            cubePlace = 1;//Right
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            cubePlace = 3;//center
                        }
                    }
                } else if (updatedRecognitions.size() == 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else //if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();

                    }


                    if (goldMineralX != -1 && silverMineral1X != -1) {
                        if (goldMineralX < silverMineral1X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            telemetry.addLine("in camera");
                            cubePlace = 5;//LEFT in camera
                        } else if (goldMineralX > silverMineral1X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            telemetry.addLine("in camera");
                            cubePlace = 4;//RIGHT in camera
                        }

                    } else {
                        telemetry.addData("Gold Mineral Position", "NOT HERE");
                        cubePlace = 0;//NOT in the camera/ only see 2 BALLS
                    }
                }
            }
            telemetry.update();

        }


        return (cubePlace);
    }


    public void followCubeRecognision(double power) {
        double runTime = 0;
        telemetry.addLine("follow cube 1:");
        telemetry.update();

        double distanceFromRight = 0;
        double distanceFromLeft = 0;
        double middleCubeX = 0;
        double k = 0.0007; //EDEN
        double[] addToMotors;
        addToMotors = new double[2];

        boolean breakLoop = false;
        runTime = getRuntime();

//        RecognitionList.get(indexGold);
        if (tfod != null)
            do {
                List<Recognition> RecognitionList = tfod.getRecognitions();// I delete List<Recognition>
                Recognition goldReco = null;

                if (RecognitionList != null) {
                    for (Recognition recognition : RecognitionList) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldReco = recognition;
                            runTime = getRuntime();
                            break;
                        }
                    }
                }

                if (goldReco != null) {

                    middleCubeX = ((goldReco.getLeft() + goldReco.getRight()) / 2);
                    distanceFromRight = 720 - middleCubeX;
                    distanceFromLeft = middleCubeX;
                    telemetry.addLine("follow cube 4:");

                    addToMotors[0] = k * distanceFromRight;  //RIGHT
                    addToMotors[1] = k * distanceFromLeft; //LEFT
                    telemetry.addData("distance From Right:", addToMotors[0]);
                    telemetry.addData("distance From Left:", addToMotors[1]);
                    telemetry.update();


                    robot.driveTrain[0][1].setPower(addToMotors[0] + power);//RIGHT Front
                    robot.driveTrain[1][1].setPower(addToMotors[0] + power);//Right Back
                    robot.driveTrain[1][0].setPower(addToMotors[1] + power);//Left Back
                    robot.driveTrain[0][0].setPower(addToMotors[1] + power);//LEFT Front
                } else {
                    telemetry.addLine("gold reco = null");
                    telemetry.update();
                    robot.driveTrain[0][1].setPower(0);//RIGHT Front
                    robot.driveTrain[1][1].setPower(0);//Right Back
                    robot.driveTrain[1][0].setPower(0);//Left Back
                    robot.driveTrain[0][0].setPower(0);//LEFT Front
                    if ((runTime - getRuntime()) < -2) {

                        breakLoop = true;
                        telemetry.addLine("in 2 seconds dont see the cube");
                        telemetry.update();
                    }
                }


            }
            while (opModeIsActive() && !breakLoop);
        robot.driveTrain[0][1].setPower(0);//RIGHT Front
        robot.driveTrain[1][1].setPower(0);//Right Back
        robot.driveTrain[1][0].setPower(0);//Left Back
        robot.driveTrain[0][0].setPower(0);//LEFT Front
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
        Orientation axis = getAxis(imu);
        return axis.secondAngle > RolltargetAngleMin && axis.secondAngle < RolltargetAngleMax && axis.thirdAngle > PitchtargetAngleMin && axis.thirdAngle < PitchtargetAngleMax;

    }

    public Orientation getAxis(BNO055IMU imu) {
        return imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void ResetHue(ColorSensor color, float[] hsvArr) { //Reset the sensor color to read bt hue values.
        Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                (int) (color.green() * SCALE_FACTOR),
                (int) (color.blue() * SCALE_FACTOR),
                hsvArr);
    }

    public void setMotorPower(double[][] power) { //Stores the four drivetrain motors power in array
        for (int row = 0; opModeIsActive() && row < 2; row++)
            for (int col = 0; opModeIsActive() && col < 2; col++)
                robot.driveTrain[row][col].setPower(power[row][col]);
    }

    public void setMotorSHAFTPower(double power) { //Stores the four drivetrain motors power in array
        for (int row = 0; opModeIsActive() && row < 2; row++)
            robot.shaft[row].setPower(power);
    }

    public void straightOnLine(int color, double power) {

        ResetHue(robot.colorRightFront, robot.valuesRightFront);
        ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
        telemetry.addData("hsvValues[0]", robot.valuesRightFront[0]);
        telemetry.update();
        if (color == 0) {

            double time = getRuntime();
            while (opModeIsActive() && robot.valuesRightFront[0] > robot.redColorRightSensor && robot.valuesLeftFront[0] > robot.redColorLeftSensor && (time + 1.5 > getRuntime())) {
                ResetHue(robot.colorRightFront, robot.valuesRightFront);
                ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
                setMotorPower(new double[][]{{power, power}, {power, power}});
                telemetry.addLine("search the First Line");
                telemetry.addData("hsvValuesRightFront[0]", robot.valuesRightFront[0]);
                telemetry.update();
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});

            ResetHue(robot.colorRightFront, robot.valuesRightFront);
            ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
            if (robot.valuesRightFront[0] < robot.redColorRightSensor) {
                while (robot.valuesLeftFront[0] > robot.redColorLeftSensor && opModeIsActive()) {
                    ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
                    setMotorPower(new double[][]{{0.75 * power, 0}, {0.75 * power, 0}});
                    telemetry.addData("search the left Line", robot.valuesLeftFront[0]);
                    telemetry.update();
                }
            }
            if (robot.valuesLeftFront[0] < robot.redColorLeftSensor) {
                while (robot.valuesRightFront[0] > robot.redColorRightSensor && opModeIsActive()) {
                    ResetHue(robot.colorRightFront, robot.valuesRightFront);
                    setMotorPower(new double[][]{{0, 0.75 * power}, {0, 0.75 * power}});
                    telemetry.addData("search the Right Line", robot.valuesRightFront[0]);
                    telemetry.update();
                }
            }

        }
        if (color == 1) {

            double time = getRuntime();
            while (opModeIsActive() && robot.valuesRightFront[0] > robot.blueColorRightSensor && robot.valuesLeftFront[0] > robot.blueColorLeftSensor && (time + 1.5 > getRuntime())) {
                ResetHue(robot.colorRightFront, robot.valuesRightFront);
                ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
                setMotorPower(new double[][]{{power, power}, {power, power}});
                telemetry.addLine("search the First Line");
                telemetry.addData("hsvValuesRightFront[0]", robot.valuesRightFront[0]);
                telemetry.update();
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});

            ResetHue(robot.colorRightFront, robot.valuesRightFront);
            ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
            if (robot.valuesRightFront[0] < robot.redColorRightSensor) {
                while (robot.valuesLeftFront[0] > robot.redColorLeftSensor && opModeIsActive()) {
                    ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
                    setMotorPower(new double[][]{{0.75 * power, 0}, {0.75 * power, 0}});
                    telemetry.addData("search the left Line", robot.valuesLeftFront[0]);
                    telemetry.update();
                }
            }
            if (robot.valuesLeftFront[0] < robot.redColorLeftSensor) {
                while (robot.valuesRightFront[0] > robot.redColorRightSensor && opModeIsActive()) {
                    ResetHue(robot.colorRightFront, robot.valuesRightFront);
                    setMotorPower(new double[][]{{0, 0.75 * power}, {0, 0.75 * power}});
                    telemetry.addData("search the Right Line", robot.valuesRightFront[0]);
                    telemetry.update();
                }
            }

        }
    }

    public void initVuforiaPhoneCamera() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = (VuforiaLocalizerEx) ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    public void initVuforiaWebCam(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine

        vuforia = (VuforiaLocalizerEx) ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod(HardwareMap hardwareMap) {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        // tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }

//    public void driveByEncoderRoverRuckus(int goalDistRight, int goalDistLeft, double power) {// Drive by encoders and converts incoders ticks to distance in cm and drives until distance is completed.
////direction 0 is forword, 1 is backword
//        final int tixRound = 600;
//        final int cmRound = 27;
//
//
//        int dRight = (goalDistRight * tixRound) / cmRound;
//        int dLeft = (goalDistLeft * tixRound) / cmRound;
////§ ß α
//
//        robot.driveTrain[0][0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.driveTrain[1][0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.driveTrain[0][1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.driveTrain[1][1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        robot.driveTrain[0][0].setTargetPosition(robot.driveTrain[0][0].getCurrentPosition() + dLeft);
//        robot.driveTrain[1][0].setTargetPosition(robot.driveTrain[1][0].getCurrentPosition() + dLeft);
////        robot.driveTrain[0][1].setTargetPosition(robot.driveTrain[0][1].getCurrentPosition() + dRight);
////        robot.driveTrain[1][1].setTargetPosition(robot.driveTrain[1][1].getCurrentPosition() + dRight);
//
//
//        robot.driveTrain[0][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain[1][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.driveTrain[0][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.driveTrain[1][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        for (int i = 0; i < 2; i++)
//            for (int j = 0; j < 2; j++)
//                robot.driveTrain[i][j].setPower(power);
//
//        telemetry.addLine("go to target");
//        telemetry.update();
//
//        double runTime = getRuntime();
//        while (opModeIsActive() &&
//                robot.driveTrain[0][0].isBusy()
//                && robot.driveTrain[1][0].isBusy()
////                && robot.driveTrain[0][1].isBusy()
////                && robot.driveTrain[1][1].isBusy()
//                && getRuntime() - runTime < Math.abs((dRight + dLeft / 2) / 10)) {
//            sleep(0);
//        }
//
//
//        for (int i = 0; i < 2; i++)
//            for (int j = 0; j < 2; j++)
//                robot.driveTrain[i][j].setPower(0);
//
//        telemetry.addLine("end move encoder");
//        telemetry.update();
//
//
//        robot.driveTrain[0][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.driveTrain[1][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.driveTrain[0][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.driveTrain[1][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//    }

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
        while (opModeIsActive() &&
                robot.shaft[0].isBusy() &&
                robot.shaft[1].isBusy()) {
            telemetry.addData("shaft move  ", robot.shaft[0].isBusy());
            telemetry.update();
        }
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
    }

    public void Parking(int targetPositionEncoder, int targetShaftParkingPositionEncoder, double shaftPower, int linearTargetEncoder, double linearPower) {
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveUtils.driveByEncoderRoverRuckus(160, 160, 0.5, false);
        robot.shaft[0].setTargetPosition(targetPositionEncoder);//250
        robot.shaft[1].setTargetPosition(targetPositionEncoder);

        robot.shaft[0].setTargetPosition(750);
        robot.shaft[1].setTargetPosition(750);
        robot.shaft[0].setPower(0.6);
        robot.shaft[1].setPower(0.6);
        while (robot.shaft[0].isBusy() && robot.shaft[1].isBusy()) ;
        robot.linear.setTargetPosition(-750);
        while (robot.linear.isBusy()) ;

//        robot.shaft[0].setTargetPosition(targetShaftParkingPositionEncoder);//250
//        robot.shaft[1].setTargetPosition(targetShaftParkingPositionEncoder);
//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.shaft[0].setPower(shaftPower);
//        robot.shaft[1].setPower(shaftPower);
//
//        robot.shaft[0].setTargetPosition(linearTargetEncoder);
//
//        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.linear.setPower(linearPower);
//        while (opModeIsActive() &&
//                robot.linear.isBusy()) {
//            telemetry.addData("linear move", robot.linear.isBusy());
//            telemetry.update();
//        }

    }


    public void ScaledTurn(double goalAngle, DcMotor[][] driveMotors, BNO055IMU imu,
                           double power) {
        boolean sideOfTurn = true;
        double deltaAngle = 0;

        boolean directTurn = true;
        double currentAngle = getCurrentScaledAngle();
        double angle0 = currentAngle;
        if (currentAngle < goalAngle) {
            if (goalAngle - currentAngle <= 360 - (goalAngle - currentAngle)) {
                sideOfTurn = false;
                deltaAngle = goalAngle - currentAngle;
            } else {
                sideOfTurn = true;
                deltaAngle = 360 - (goalAngle - currentAngle);
                directTurn = false;
            }


        } else {
            if (currentAngle - goalAngle <= 360 - (currentAngle - goalAngle)) {
                sideOfTurn = true;
                deltaAngle = currentAngle - goalAngle;
            } else {
                sideOfTurn = false;
                deltaAngle = 360 - (currentAngle - goalAngle);
                directTurn = false;
            }
        }
        if (sideOfTurn)
            setMotorPower(new double[][]{{power, -power}, {power, -power}});
        else
            setMotorPower(new double[][]{{-power, power}, {-power, power}});
        if (directTurn)
            while (opModeIsActive() && Math.abs(angle0 - currentAngle) < deltaAngle) {  //motors running
                currentAngle = getCurrentScaledAngle();
                telemetry.addData("angle case 3:", currentAngle);
                telemetry.update();
            }
        else if (goalAngle > 180 && currentAngle < 180)
            while (opModeIsActive() &&
                    (currentAngle <= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle > 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                currentAngle = getCurrentScaledAngle();
                telemetry.addData("angle case 1:", currentAngle);
                telemetry.update();
            }

        else if (goalAngle < 180 && currentAngle > 180)
            while (opModeIsActive() && (currentAngle >= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle < 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                currentAngle = getCurrentScaledAngle();
                telemetry.addData("angle case 2:", currentAngle);
                telemetry.update();
            }


        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }

    public static double normalizedAngle(double angle) {
        if (angle < 0) {
            while (angle < 0)
                angle += 360;
        } else if (angle >= 360) {
            while (angle >= 360)
                angle -= 360;
        }
        return angle;
    }

    public void driveToImage(double power) {
        //  Driving.Driving.setMotorPower(motors, new double[][]{{0.23, 0.23}, {0.23, 0.23}});
        float[] positions = getPositions();//למה לקרוא פעמים לאותה הפונקציה?
        if (positions != null) {

            sleep(1000);
            setMotorPower(new double[][]{{power, power}, {power, power}});
            while (opModeIsActive() && positions[0] <= 48) {
                positions = getPositions();
                telemetry.addData("x:", positions[0]);
                telemetry.update();
            }

            telemetry.addLine("got to x=65");
            telemetry.update();
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            sleep(1000);

            diffTurn(90 - positions[5], 0.15);
        }
    }

    public void diffTurn(double diffAngle, double power) {
        double currAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double goalAngle = normalizedAngle(diffAngle + currAngle);
        ScaledTurn(goalAngle, robot.driveTrain, robot.imu, 0.4);

    }

    public float[] getPositions() {
        for (VuforiaTrackable trackable : allTrackablesNav) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
        // Provide feedback as to where the robot is located (if we know).
        if (lastLocation != null) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();

            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            /*
             *0- x
             *1- y
             *2- z
             *3-roll
             *4-pitch
             *5-heading*/
            return new float[]{translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch, rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle};
            // express the rotation of the robot in degrees.
        }
        return null;
    }

    public void searchImage(int cubePos, double power) {
        runtime.reset();//TODO: delete this
        double time0 = runtime.seconds();
        double currTime = time0;

        int count = 2;
        double maxTime = 5;
        if (cubePos == 1) {
            maxTime += 0.6;
            count = 0;
        } else if (cubePos == 2) {
            maxTime += 0.3;
            count = 1;
        }


        boolean per = true;
        while (opModeIsActive() && currTime - time0 < maxTime && getPositions() == null /*&& count < 10*/) {
            if (per) {
                setMotorPower(new double[][]{{power, power - 0.17}, {power, power - 0.17}});
                telemetry.addLine("side 1");
                telemetry.update();
            } else {
                setMotorPower(new double[][]{{power - 0.17, power}, {power - 0.17, power}});
                telemetry.addLine("side 2");
                telemetry.update();
            }
            currTime = runtime.seconds();
            if (currTime - time0 >= 0.28) {
                runtime.reset();
                count++;
                per = !per;
                setMotorPower(new double[][]{{0, 0}, {0, 0}});
                sleep(25);

            }

            telemetry.addData("time passed: ", currTime - time0);
            telemetry.update();
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }

    public double getCurrentScaledAngle() {
        BNO055IMU imu = robot.imu;
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle < 0)
            angle += 360;
        return angle;
    }


}