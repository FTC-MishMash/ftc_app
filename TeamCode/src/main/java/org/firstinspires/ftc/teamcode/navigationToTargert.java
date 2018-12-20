package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by user on 06/11/2018.
 */
@Autonomous(name = "ImageTesting")
public class navigationToTargert extends LinearOpMode {
    Robot robot;
    DcMotor[][] motors;
    double power = -0.17;
    final double minAngleToTarget = 35;
    static final int XtargetPosition = 63;
    static final int YtargetPosition = 6;
    static final int ZtargetPosition = -4;
    static final int PitchtargetAngleMin = -10;
    static final int PitchtargetAngleMax = 30;
    static final int HeadingToSampling = 45;
    static final int HeadingToTarget = 90;
    static final int RolltargetAngleMin = 0;
    static final int RolltargetAngleMax = 0;
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final String VUFORIA_KEY = " ATgDONj/////AAABmW0G/nQirUMiumnzPc6Pl8oJhBOCC2qoUq0BWhir9YWcBFDlhZUfSwATcQArcyyLxIOV21sHaYJeeQEJZfIJ+4spBn3oJ/DfycsbPaNs87+TRpM46/vbUkj1Ok+NtZ/eqMhmMXjFC8dgdCfbCt0aMxoBNzDw4+v28abG+hjUCjVYf86Jq1m7R942XCjw0yhOZqTXWIp3WAZDXY/PdWGQGY/zWae0l6TAZ6Z27t1xYJdkkpLqEsbKM3ZprvtgIs8AsWS9Tri2892OHq2CnCL+1ZHHXKPdxON3fiC1Gd3oihwPhTUReNw0VAg9yeVsVa1UQg7ea9K6WpmVto0FG+T2/LV8uq/3Mp/NHWiNizw2DM4h";
    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    ElapsedTime runtime = new ElapsedTime();
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    List<VuforiaTrackable> allTrackablesNav;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    BNO055IMU imu;
    final double tixRound = 600;
    final double cmRound = 27;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        motors = robot.driveTrain;
        imu = robot.imu;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

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
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();


        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
        waitForStart();

    while (opModeIsActive()){
        telemetry.addData("angle: ",Driving.getCurrentScaledAngle(imu));
        telemetry.update();
    }
//
//        setMotorPower(motors, new double[][]{{power, power}, {power, power}});
//        searchImage();
//        setMotorPower(motors, new double[][]{{power, -power}, {power, -power}});
//        while (getPositions() == null)
//            setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});
//        sleep(1000);///j
//        driveToImage();
    }




    /**
     * Driving the robot near the target image and turn it across the depot.
     */
    private void driveToImage() {

        //  Driving.setMotorPower(motors, new double[][]{{0.23, 0.23}, {0.23, 0.23}});
        float[] positions = getPositions();
        if (positions != null) {
            setMotorPower(motors, new double[][]{{-power, power}, {-power, power}});
            while (opModeIsActive() && positions[5] >= 100) {
                positions = getPositions();
                telemetry.addData("heading:", positions[5]);
                telemetry.update();
            }
            setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});
            sleep(1000);
            setMotorPower(motors, new double[][]{{power, power}, {power, power}});
            while (opModeIsActive() && positions[0] <= 60) {
                positions = getPositions();
                telemetry.addData("x:", positions[0]);
                telemetry.update();
            }

            telemetry.addLine("got to x=65");
            telemetry.update();
            setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});
            sleep(1000);
            setMotorPower(motors, new double[][]{{0.23, -0.23}, {0.23, -0.23}});
            while (opModeIsActive() && positions[5] >= 94) {
                positions = getPositions();
                telemetry.addData("heading:", positions[5]);
                telemetry.update();
            }
            setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});

        }
    }

    public void setMotorPower(DcMotor[][] motors, double[][] power) { //Stores the four motors motors power in array
        for (int row = 0; opModeIsActive() && row < 2; row++)
            for (int col = 0; opModeIsActive() && col < 2; col++)
                motors[row][col].setPower(power[row][col]);
    }
    public  void ScaledTurn(double goalAngle, DcMotor[][] driveMotors, BNO055IMU imu, double power) {

        //  autoMode auto;
        double currentAngle = Driving.getCurrentScaledAngle(imu);
        double angle0 = currentAngle;
        Driving.setTurnDirection(currentAngle, goalAngle);
        if (sideOfTurn)
            setMotorPower(driveMotors, new double[][]{{power, -power}, {power, -power}});
        else
            setMotorPower(driveMotors, new double[][]{{-power, power}, {-power, power}});
        if (directTurn)
            while (Math.abs(angle0 - currentAngle) < deltaAngle) {  //motors running
                currentAngle = getCurrentScaledAngle(imu);
                telemetry.addData("angle case 3:", currentAngle);
                telemetry.update();
            }
        else if (goalAngle > 180 && currentAngle < 180)
            while (
                    (currentAngle <= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle > 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                currentAngle = getCurrentScaledAngle(imu);
                telemetry.addData("angle case 1:", currentAngle);
                telemetry.update();
            }

        else if (goalAngle < 180 && currentAngle > 180)
            while ((currentAngle >= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle < 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                currentAngle = getCurrentScaledAngle(imu);
                telemetry.addData("angle case 2:", currentAngle);
                telemetry.update();
            }


        setMotorPower(driveMotors, new double[][]{{0, 0}, {0, 0}});
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

    public void searchImage() {
        runtime.reset();
        double time0 = runtime.seconds();
        double currTime = time0;
        while (opModeIsActive() && currTime - time0 < 5 && getPositions() == null) {
            currTime = runtime.seconds();
            telemetry.addData("time passed", currTime - time0);
            telemetry.update();
        }
    }

    public void driveTargetImageEncoder(double goalDist, double direction, double k,
                                        double heading) {// Drive by encoders and converts incoders ticks to distance in cm and drives until distance is completed.
        if (opModeIsActive()) {
//dc motor [0][0] not working
            // k is power for motors
            //Reset encoders
            motors[1][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[1][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[0][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[0][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[1][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double pidErr[] = {0, 0};
//PID reset
            pidErr = PID.GyroPID(imu, heading, pidErr[1]); //Convert degrees to radians
            double deg2rad = direction / 180 * Math.PI;
            double currDist = 0;
            double err = goalDist;
            double time0 = getRuntime();
// Drive until distance target completed

            while (opModeIsActive() && getPositions() == null && (err) > 2 && (getRuntime() - time0) < goalDist / 10) {
                if (err > 0) {
                    motors[0][0].setPower(k * (Math.cos(deg2rad) + Math.sin(deg2rad) - pidErr[0]));
                    motors[1][0].setPower(k * (Math.cos(deg2rad) - Math.sin(deg2rad) + pidErr[0]));
                    motors[0][1].setPower(k * (Math.cos(deg2rad) - Math.sin(deg2rad)) - pidErr[0]);
                    motors[1][1].setPower(k * (Math.cos(deg2rad) + Math.sin(deg2rad)) + pidErr[0]);
                }

                currDist = Math.abs((
                                (motors[1][0].getCurrentPosition() * cmRound / tixRound) +
                                        (motors[0][1].getCurrentPosition() * cmRound / tixRound) +
                                        (motors[1][1].getCurrentPosition() * cmRound / tixRound)
                        ) / 3
                );
                err = goalDist - currDist;
                telemetry.addData("currDist", currDist);
                telemetry.addData("motors[1][0]", (motors[1][0].getCurrentPosition()));
                telemetry.addData("motors[0][1]", (motors[0][1].getCurrentPosition()));
                telemetry.addData("motors[1][1]", (motors[1][1].getCurrentPosition()));
                telemetry.addData("current distance: ", currDist);
                telemetry.addData("current error: ", err);
                telemetry.update();
            }
        }
        setMotorPower(motors, new double[][]{{0.0, 0.0}, {0.0, 0.0}});

    }

}
