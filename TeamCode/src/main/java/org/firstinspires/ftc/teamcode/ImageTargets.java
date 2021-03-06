package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.DriveUtilities.setMotorPower;

public class ImageTargets {
    Robot robot;
    AutoMode currOpmode;
    public VuforiaLocalizerEx vuforia;
    Telemetry telemetry;
    DriveUtilities driveUtilities;
    float ypos = 56.2f;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;
    public ElapsedTime runTime = new ElapsedTime();
    final double SCALE_FACTOR = 255;
    int distImage = 35;//inbar change this


    DcMotor[][] motors;
    int imageSearchDist = -25;
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


    public ImageTargets(AutoMode currOpmode) {
        this.currOpmode = currOpmode;
        this.robot = currOpmode.robot;
        this.vuforia = currOpmode.vuforia;
        this.telemetry = currOpmode.telemetry;
        this.motors = robot.driveTrain;
        this.driveUtilities = currOpmode.driveUtils;
    }

    public void startTracking() {

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


        final int CAMERA_FORWARD_DISPLACEMENT = 300;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 250;   // eg: Camera is 200 mm above ground
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

    public float[] getPositions() {
        for (VuforiaTrackable trackable : allTrackablesNav) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            //     telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

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

    public void searchImage(TensorflowUtils.MINERAL_POSITION cubePos, double power) {
        this.driveUtilities = currOpmode.driveUtils;
        telemetry.addData("on search", currOpmode.targetNav == null);
        telemetry.update();

        switch (cubePos) {
            case RIGHT: {
                telemetry.addLine("case1");
                telemetry.update();
                driveUtilities.driveByEncoderRoverRuckus(imageSearchDist - 30, imageSearchDist - 30, power, true);
                break;
            }
            case CENTER: {
                telemetry.addLine("case2");
                telemetry.update();
                driveUtilities.driveByEncoderRoverRuckus(-48, -37, power, true);
                driveUtilities.driveByEncoderRoverRuckus(imageSearchDist - 15, imageSearchDist - 15, power, true);

                break;
            }
            case LEFT: {
                driveUtilities.driveByEncoderRoverRuckus(imageSearchDist, imageSearchDist, power, true);
            }
        }
//
//        runtime.reset();//TODO: delete this
//        double time0 = runtime.seconds();
//        double currTime = time0;
//
//        int count = 2;
//        double maxTime = 5;
//        if (cubePos == 1) {
//            maxTime += 0.6;
//            count = 0;
//        } else if (cubePos == 2) {
//            maxTime += 0.3;
//            count = 1;
//        }
//
//
//        boolean per = true;
//
//        while (currOpmode.opModeIsActive() && currTime - time0 < maxTime && getPositions() == null /*&& count < 10*/) {
//            if (per) {
//                setMotorPower(motors, new double[][]{{power, power - 0.17}, {power, power - 0.17}});
//                telemetry.addLine("side 1");
//                telemetry.update();
//            } else {
//                setMotorPower(motors, new double[][]{{power - 0.17, power}, {power - 0.17, power}});
//                telemetry.addLine("side 2");
//                telemetry.update();
//            }
//            currTime = runtime.seconds();
//            if (currTime - time0 >= 0.28) {
//                runtime.reset();
//                count++;
//                per = !per;
//                setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});
//                currOpmode.sleep(25);
//
//            }
//
//            telemetry.addData("time passed: ", currTime - time0);
//            telemetry.update();
//        }
//        setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});
    }


    public void driveToImage(double power) {
        //  Driving.Driving.setMotorPower(motors, new double[][]{{0.23, 0.23}, {0.23, 0.23}});

        float[] positions = getPositions();
        if (positions != null) {
            telemetry.addLine("On DriveToImage()");
            telemetry.update();
//            currOpmode.sleep(150);
            double time = currOpmode.getRuntime();
            telemetry.addLine("test image positions");
            telemetry.update();
//            while (!currOpmode.gamepad1.a) {
//                positions = getPositions();
//                telemetry.addData("x", positions[0]);
//                telemetry.addData("y", positions[1]);
//                telemetry.addData("z", positions[2]);
//                telemetry.update();
//            }
            setMotorPower(motors, new double[][]{{power, power}, {power, power}});
            double headingTarget = -90 * Math.signum(positions[1]);
            float totalPassed = 1;
            float firstpos = Math.abs(positions[1]);
            //float ypos = 57;//inbar change from 53 to 57
            while (currOpmode.opModeIsActive() && (positions != null && Math.abs(positions[1]) <= ypos)) {
                totalPassed = (ypos - firstpos) / (ypos - Math.abs(positions[1]));
                positions = getPositions();
                telemetry.addData("x:", positions[1]);
                telemetry.update();
            }
            if (positions == null) {
                int distToComplete = Math.round(distImage * totalPassed);
                driveUtilities.driveByEncoderRoverRuckus(-distToComplete, -distToComplete, -robot.powerEncoder, false);
            }
            telemetry.addLine("got to x=65");
            telemetry.update();
            setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});
            currOpmode.sleep(100);
            double heading = 0;
            positions = getPositions();
            if (positions != null) ;
            {
                heading = positions[5];
                telemetry.addData("got to heading: ", heading);
                telemetry.update();
                currOpmode.sleep(400);
            }
//            while (!currOpmode.gamepad1.a) {
//                positions = getPositions();
//                telemetry.addData("currIMU: ",DriveUtilities.normalizedAngle(DriveUtilities.getAngularOriention(robot.imu).firstAngle));
//                telemetry.addData("heading: ", positions[5]);
//                telemetry.update();
//            }
            //   driveUtilities.TurnWithEncoder(310,0.4);

            currOpmode.driveUtils.diffTurn(3.8+ headingTarget - heading);
//                currOpmode.sleep(2000);
            //  driveUtilities.Turn(210
        }
    }
}