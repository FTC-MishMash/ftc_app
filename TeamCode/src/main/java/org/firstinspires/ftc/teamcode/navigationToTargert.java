package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
public class navigationToTargert extends autoMode {

    double power = 0.3;
    final double tixRound = 600;
    final double cmRound = 27;
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private ElapsedTime runtime = new ElapsedTime();
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private List<VuforiaTrackable> allTrackablesNav;

    public static final String VUFORIA_KEY = " ATgDONj/////AAABmW0G/nQirUMiumnzPc6Pl8oJhBOCC2qoUq0BWhir9YWcBFDlhZUfSwATcQArcyyLxIOV21sHaYJeeQEJZfIJ+4spBn3oJ/DfycsbPaNs87+TRpM46/vbUkj1Ok+NtZ/eqMhmMXjFC8dgdCfbCt0aMxoBNzDw4+v28abG+hjUCjVYf86Jq1m7R942XCjw0yhOZqTXWIp3WAZDXY/PdWGQGY/zWae0l6TAZ6Z27t1xYJdkkpLqEsbKM3ZprvtgIs8AsWS9Tri2892OHq2CnCL+1ZHHXKPdxON3fiC1Gd3oihwPhTUReNw0VAg9yeVsVa1UQg7ea9K6WpmVto0FG+T2/LV8uq/3Mp/NHWiNizw2DM4h";
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
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
//        while (!isStarted()) {
//            telemetry.addData("angle", getPositions()[5]);
//            telemetry.update();
//        }
        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackablesNav) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */


        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();

        waitForStart();

        while (opModeIsActive()) {
            float[] pos=getPositions();
            if(pos!=null){
            telemetry.addData("angle", pos[5]);
            telemetry.update();}

//            scaledTurnImage(310, 0.35
            //  setMotorPower(new double[][]{{power, -power}, {power, -power}});

            // telemetry.addData("ad",robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
            //   AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//            float[] positions = getPositions();
////            scaledTurnImage(35,  0.6);
////            sleep(1500);
//            telemetry.addData("pos: ",getPositions()==null);
//            telemetry.update();
//            sleep(2000);

//            scaledTurnImage(90,0.4);
        }
//
//        Driving.setMotorPower(motors, new double[][]{{power, power}, {power, power}});
//        searchImage();
//        Driving.setMotorPower(motors, new double[][]{{power, -power}, {power, -power}});
//        while (getPositions() == null)
//            Driving.setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});
//        sleep(1000);///j
//        driveToImage();
    }


    /**
     * Driving the robot near the target image and turn it across the depot.
     */

    public void driveToImage() {

        //  Driving.Driving.setMotorPower(motors, new double[][]{{0.23, 0.23}, {0.23, 0.23}});
        float[] positions = getPositions();
        if (positions != null) {
//            setMotorPower(new double[][]{{-power, power}, {-power, power}});
//            while (opModeIsActive() && positions[5] >= 100) {
//                positions = getPositions();
//                telemetry.addData("heading:", positions[5]);
//                telemetry.update();
//            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            sleep(1000);
            setMotorPower(new double[][]{{power, power}, {power, power}});
            while (opModeIsActive() && positions[0] <= 60) {
                positions = getPositions();
                telemetry.addData("x:", positions[0]);
                telemetry.update();
            }

            telemetry.addLine("got to x=65");
            telemetry.update();
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            sleep(1000);
            setMotorPower(new double[][]{{0.23, -0.23}, {0.23, -0.23}});
//            while (opModeIsActive() && positions[5] >= 94) {
////                positions = getPositions();
////                telemetry.addData("heading:", positions[5]);
////                telemetry.update();
////            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            diffTurn(90-positions[5],0.4);
        }
    }


    public void scaledTurnImage(double goalAngle, double power) {
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
            while (opModeIsActive() && getPositions() == null && Math.abs(angle0 - currentAngle) < deltaAngle) {  //motors running
                currentAngle = getCurrentScaledAngle();
                telemetry.addData("angle case 3:", currentAngle);
                telemetry.update();
            }
        else if (goalAngle > 180 && currentAngle < 180)
            while (opModeIsActive() && getPositions() == null && (
                    (currentAngle <= 180 && (Math.abs(angle0 - currentAngle) < deltaAngle)) || (currentAngle > 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle))) {//motors running
                currentAngle = getCurrentScaledAngle();
                telemetry.addData("angle case 1:", currentAngle);
                telemetry.update();
            }

        else if (goalAngle < 180 && currentAngle > 180)
            while (opModeIsActive() && getPositions() == null && ((currentAngle >= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle < 180 && 360 - Math.abs(angle0 - currentAngle) < deltaAngle))) {//motors running
                currentAngle = getCurrentScaledAngle();
                telemetry.addData("angle case 2:", currentAngle);
                telemetry.update();
            }


        setMotorPower(new double[][]{{0, 0}, {0, 0}});
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

    //aa
    public void searchImage() {
        runtime.reset();
        double time0 = runtime.seconds();
        double currTime = time0;
        power = -0.32;
        int count = 0;
        boolean per = true;
        while (opModeIsActive() && currTime - time0 < 6 && getPositions() == null && count < 10) {
            if (per) {
                setMotorPower(new double[][]{{power - 0.21, power}, {power - 0.21, power}});
                telemetry.addLine("side 1");
                telemetry.update();
            } else {
                setMotorPower(new double[][]{{power, power - 0.21}, {power, power - 0.21}});
                telemetry.addLine("side 2");
                telemetry.update();
            }
            currTime = runtime.seconds();
            if (currTime - time0 >= 0.3) {
                runtime.reset();
                count++;
                per = !per;
            }
            telemetry.addData("time passed: ", currTime - time0);
            telemetry.update();
        }
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

    public void diffTurn(double diffAngle, double power) {
        double currAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double goalAngle = normalizedAngle(diffAngle + currAngle);
        ScaledTurn(goalAngle, robot.driveTrain, robot.imu, 0.4);

    }

}

