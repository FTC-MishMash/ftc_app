package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


/**
 * Created by user on 22/11/2018.
 */
@Autonomous(name = "AutoMode")
public class autoMode extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    //    private static final android.graphics.Color Color = ;
    Robot robot;
    public static final String VUFORIA_KEY = " ATgDONj/////AAABmW0G/nQirUMiumnzPc6Pl8oJhBOCC2qoUq0BWhir9YWcBFDlhZUfSwATcQArcyyLxIOV21sHaYJeeQEJZfIJ+4spBn3oJ/DfycsbPaNs87+TRpM46/vbUkj1Ok+NtZ/eqMhmMXjFC8dgdCfbCt0aMxoBNzDw4+v28abG+hjUCjVYf86Jq1m7R942XCjw0yhOZqTXWIp3WAZDXY/PdWGQGY/zWae0l6TAZ6Z27t1xYJdkkpLqEsbKM3ZprvtgIs8AsWS9Tri2892OHq2CnCL+1ZHHXKPdxON3fiC1Gd3oihwPhTUReNw0VAg9yeVsVa1UQg7ea9K6WpmVto0FG+T2/LV8uq/3Mp/NHWiNizw2DM4h";
    double power = 0.19;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;


    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    ElapsedTime runTime = new ElapsedTime();
    final double SCALE_FACTOR = 255;

    static final int PitchtargetAngleMin = -5;
    static final int PitchtargetAngleMax = 5;
    static final int RolltargetAngleMin = -10;
    static final int RolltargetAngleMax = 10;


    final double minAngleToTarget = 35;
    static final int XtargetPosition = 63;
    static final int YtargetPosition = 6;
    static final int ZtargetPosition = -4;

    static final int HeadingToSampling = 45;
    static final int HeadingToTarget = 90;

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    ElapsedTime runtime = new ElapsedTime();
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    List<VuforiaTrackable> allTrackablesNav;
    BNO055IMU imu;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
//        robot.shaft[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.shaft[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.linear[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        initVuforiaWebCam();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        if (tfod != null) {
            tfod.activate();
        }
        int cubePlace = -1;//dont see any cube.
        while (!isStarted()) {
            cubePlace = getCube();//update cube location
            sleep(2);
        }
        if (tfod != null) {
            tfod.deactivate();
        }

        waitForStart();
//        int cubePlace = -1;//dont see any cube
//            cubePlace = getCube();//update cube location
        runTime.reset();
        runTime.startTime();
        if (opModeIsActive()) {

//        getOffTheClimb(robot.imu, robot.shaft, 0.3);

            if (cubePlace == -1) {//there is NOT cube/ or only one ball

            } else if (cubePlace == 0) {//see only 2 balls

                ScaledTurn(50, robot.driveTrain, robot.imu, 0.5);

            } else if (cubePlace == 1) {//cube RIGHT

                ScaledTurn(15, robot.driveTrain, robot.imu, 0.5);

            } else if (cubePlace == 2) {//cube LEFT

                ScaledTurn(70, robot.driveTrain, robot.imu, 0.5);

            } else if (cubePlace == 3) {//cube CENTER
                //No need to move


            } else if (cubePlace == 4) {//cube RIGHT in camera
//TODO: add what the robot need to do in this
//            Driving.ScaledTurn(50,motors,robot.imu,0.5,telemetry);

            } else if (cubePlace == 5) {//cube LEFT in camera
//TODO: add what the robot need to do in this
//            Driving.ScaledTurn(50,motors,robot.imu,0.5,telemetry);

            }
            if (tfod != null) {
                tfod.activate();
            }
            followCubeRecognision(0.15);//start power
            if (tfod != null) {
                tfod.shutdown();
            }
            driveByEncoderRoverRuckus(7, 7, 0.5);
            sleep(2000);
            driveByEncoderRoverRuckus(-20, -20, 0.5);
            sleep(2000);
            ScaledTurn(60, robot.driveTrain, robot.imu, 0.5);

        }
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

    public void startTracking(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //   Robot robot=new Robot(hardwareMap);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
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

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackablesNav) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "c");
        telemetry.update();


        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
    }

    public int getCube() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first..
//        initVuforia();
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }
//
//        if (tfod != null) {
//            tfod.activate();
//        }
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
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {//TODO: add cibe place
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


//    private void followCube(double power) {
//        double runTime = 0;
//        telemetry.addLine("follow cube 1:");
//        telemetry.update();
//
//        double distanceFromRight = 0;
//        double distanceFromLeft = 0;
//        double middleCubeX = 0;
//        double k = 0.0007; //EDEN
//        double[] addToMotors;
//        addToMotors = new double[2];
//        boolean firstGold = false;
//        boolean breakLoop = false;
//
//
////        while (opModeIsActive()) {//TODO: ADD CONDITION
////            telemetry.addLine("search cube 1");
////            telemetry.update();
////            if (tfod != null)
////                updatedRecognitions = tfod.getUpdatedRecognitions();
////
////            if (updatedRecognitions != null
////               &&!updatedRecognitions.isEmpty()
////                        && updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL))
////                    break;
////        }
//        if (tfod != null)
//            do {
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//
//
//                if (updatedRecognitions != null
//                        && !updatedRecognitions.isEmpty()//was changed
//                    //     && updatedRecognitions.get(0) != null
//                        ) {
//                    if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
//                        firstGold = true;
//                        middleCubeX = ((updatedRecognitions.get(0).getLeft() + updatedRecognitions.get(0).getRight()) / 2);
//                        distanceFromRight = 700 - middleCubeX;
//                        distanceFromLeft = middleCubeX;
//                        telemetry.addLine("follow cube 4:");
//
//                        addToMotors[0] = k * distanceFromRight;  //RIGHT
//                        addToMotors[1] = k * distanceFromLeft; //LEFT
//                        telemetry.addData("distance From Right:", addToMotors[0]);
//                        telemetry.addData("distance From Left:", addToMotors[1]);
//                        telemetry.update();
//
//
//                        robot.driveTrain[0][1].setPower(addToMotors[0] + power);//RIGHT Front
//                        robot.driveTrain[1][1].setPower(addToMotors[0] + power);//Right Back
//                        robot.driveTrain[1][0].setPower(addToMotors[1] + power);//Left Back
//                        robot.driveTrain[0][0].setPower(addToMotors[1] + power);//LEFT Front
//                        runTime = getRuntime();
//
//                    } else {
//                        if (firstGold)
//                            breakLoop = true;
//                        telemetry.addLine("dont see cube 1");
//                        telemetry.update();
//                    }
//                } else {
//                    telemetry.addLine("dont see cube 2");
//                    telemetry.update();
//                    robot.driveTrain[0][1].setPower(0);//RIGHT Front
//                    robot.driveTrain[1][1].setPower(0);//Right Back
//                    robot.driveTrain[1][0].setPower(0);//Left Back
//                    robot.driveTrain[0][0].setPower(0);//LEFT Front
//                    if ((runTime - getRuntime()) < -2) {
//
//                        breakLoop = true;
//                        telemetry.addData("in 2 seconds dont see the cube    ", breakLoop);
//                        telemetry.update();
//                    }
//                    //       break;
//                }
//
////            powerAdd ToMotors[0] = getPowerMotor()[0];//RIGHT
////            powerAddToMotors[1] = getPowerMotor()[1];//LEFT
//
//
//                //   break;
//
//
//                //    }
//
//
//            }
//            while (opModeIsActive() && !breakLoop);
//        robot.driveTrain[0][1].setPower(0);//RIGHT Front
//        robot.driveTrain[1][1].setPower(0);//Right Back
//        robot.driveTrain[1][0].setPower(0);//Left Back
//        robot.driveTrain[0][0].setPower(0);//LEFT Front
//    }

    private void followCubeRecognision(double power) {
        double runTime = 0;
        telemetry.addLine("follow cube 1:");
        telemetry.update();

        double distanceFromRight = 0;
        double distanceFromLeft = 0;
        double middleCubeX = 0;
        double k = 0.0007; //EDEN
        double[] addToMotors;
        addToMotors = new double[2];
        boolean firstGold = false;
        boolean breakLoop = false;

        if (tfod != null)
            do {
                List<Recognition> RecognitionList = tfod.getRecognitions();


                if (RecognitionList != null
                        && !RecognitionList.isEmpty()//was changed
                    //     && updatedRecognitions.get(0) != null
                        ) {
                    if (RecognitionList.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                        firstGold = true;
                        middleCubeX = ((RecognitionList.get(0).getLeft() + RecognitionList.get(0).getRight()) / 2);
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
                        runTime = getRuntime();

                    } else {
                        if (firstGold)
                            breakLoop = true;
                        telemetry.addLine("dont see cube 1");
                        telemetry.update();
                    }
                } else {
                    telemetry.addLine("dont see cube 2");
                    telemetry.update();
                    robot.driveTrain[0][1].setPower(0);//RIGHT Front
                    robot.driveTrain[1][1].setPower(0);//Right Back
                    robot.driveTrain[1][0].setPower(0);//Left Back
                    robot.driveTrain[0][0].setPower(0);//LEFT Front
                    if ((runTime - getRuntime()) < -2) {

                        breakLoop = true;
                        telemetry.addData("in 2 seconds dont see the cube    ", breakLoop);
                        telemetry.update();
                    }
                    //       break;
                }

//            powerAdd ToMotors[0] = getPowerMotor()[0];//RIGHT
//            powerAddToMotors[1] = getPowerMotor()[1];//LEFT


                //   break;


                //    }


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

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    public void initVuforiaWebCam() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void driveByEncoderRoverRuckus(int goalDistRight, int goalDistLeft, double power) {// Drive by encoders and converts incoders ticks to distance in cm and drives until distance is completed.
//direction 0 is forword, 1 is backword
        final int tixRound = 600;
        final int cmRound = 27;
        double runTime = getRuntime();
        int startCurrentPosision[][] = new int[2][2];
        int dRight = (goalDistRight * tixRound) / cmRound;
        int dLeft = (goalDistLeft * tixRound) / cmRound;
        startCurrentPosision[0][0] = robot.driveTrain[0][0].getCurrentPosition();
        startCurrentPosision[1][0] = robot.driveTrain[1][0].getCurrentPosition();
        startCurrentPosision[0][1] = robot.driveTrain[0][1].getCurrentPosition();
        startCurrentPosision[1][1] = robot.driveTrain[1][1].getCurrentPosition();


        robot.driveTrain[0][0].setTargetPosition(robot.driveTrain[0][0].getCurrentPosition() + dLeft);
        robot.driveTrain[1][0].setTargetPosition(robot.driveTrain[1][0].getCurrentPosition() + dLeft);
        robot.driveTrain[0][1].setTargetPosition(robot.driveTrain[0][1].getCurrentPosition() + dRight);
        robot.driveTrain[1][1].setTargetPosition(robot.driveTrain[1][1].getCurrentPosition() + dRight);


        robot.driveTrain[0][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveTrain[1][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveTrain[0][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveTrain[1][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                robot.driveTrain[i][j].setPower(power);

        telemetry.addLine("go to target");
        telemetry.update();
        while (opModeIsActive() &&
                (Math.abs((startCurrentPosision[0][0] + dLeft) - robot.driveTrain[0][0].getCurrentPosition())) > 1
                && (Math.abs((startCurrentPosision[1][0] + dLeft) - robot.driveTrain[1][0].getCurrentPosition())) > 1
                && (Math.abs((startCurrentPosision[0][1] + dRight) - robot.driveTrain[0][1].getCurrentPosition())) > 1
                && (Math.abs((startCurrentPosision[1][1] + dRight) - robot.driveTrain[1][1].getCurrentPosition())) > 1
                && getRuntime() - runTime < Math.abs((dRight + dLeft / 2) / 10)) ;


//        while (opModeIsActive() &&
//                robot.driveTrain[0][0].isBusy()
//                && robot.driveTrain[1][0].isBusy()
//                && robot.driveTrain[0][1].isBusy()
//                && robot.driveTrain[1][1].isBusy()
//                && getRuntime() - runTime < Math.abs((dRight + dLeft / 2) / 10))
//            sleep(0);

        telemetry.addLine("end move encoder");
        telemetry.update();
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                robot.driveTrain[i][j].setPower(0);

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++) {
                robot.driveTrain[i][j].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveTrain[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


    }


    public void ScaledTurn(double goalAngle, DcMotor[][] driveMotors, BNO055IMU imu, double power) {
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


    public double getCurrentScaledAngle() {
        BNO055IMU imu = robot.imu;
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle < 0)
            angle += 360;
        return angle;
    }

    public void initRobot(HardwareMap hardwareMap1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap1;
        this.telemetry = telemetry;
        robot = new Robot(hardwareMap);
    }
}