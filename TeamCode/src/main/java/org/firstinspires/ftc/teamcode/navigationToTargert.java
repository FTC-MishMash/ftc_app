package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Created by user on 06/11/2018.
 */
@Autonomous(name = "ImageTesting")
public class navigationToTargert extends LinearOpMode {

    double power = 0.3;
    final double tixRound = 600;
    final double cmRound = 27;
    autoMode auto = new autoMode();
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
    public static final String VUFORIA_KEY = " ATgDONj/////AAABmW0G/nQirUMiumnzPc6Pl8oJhBOCC2qoUq0BWhir9YWcBFDlhZUfSwATcQArcyyLxIOV21sHaYJeeQEJZfIJ+4spBn3oJ/DfycsbPaNs87+TRpM46/vbUkj1Ok+NtZ/eqMhmMXjFC8dgdCfbCt0aMxoBNzDw4+v28abG+hjUCjVYf86Jq1m7R942XCjw0yhOZqTXWIp3WAZDXY/PdWGQGY/zWae0l6TAZ6Z27t1xYJdkkpLqEsbKM3ZprvtgIs8AsWS9Tri2892OHq2CnCL+1ZHHXKPdxON3fiC1Gd3oihwPhTUReNw0VAg9yeVsVa1UQg7ea9K6WpmVto0FG+T2/LV8uq/3Mp/NHWiNizw2DM4h";
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
//        motors = robot.driveTrain;
//
        auto.initRobot(hardwareMap, telemetry);
        auto.startTracking(hardwareMap);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        //   Robot robot=new Robot(hardwareMap);
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = CAMERA_CHOICE;
//        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Load the data sets that for the trackable objects. These particular data
//        // sets are stored in the 'assets' part of our application.
//        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
//        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
//        blueRover.setName("Blue-Rover");
//        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
//        redFootprint.setName("Red-Footprint");
//        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
//        frontCraters.setName("Front-Craters");
//        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
//        backSpace.setName("Back-Space");
//
//        // For convenience, gather together all the trackable objects in one easily-iterable collection */
//        allTrackablesNav = new ArrayList<VuforiaTrackable>();
//        allTrackablesNav.addAll(targetsRoverRuckus);
//
//
//        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
//                .translation(0, mmFTCFieldWidth, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
//        blueRover.setLocation(blueRoverLocationOnField);
//
//
//        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
//                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
//        redFootprint.setLocation(redFootprintLocationOnField);
//
//
//        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
//                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
//        frontCraters.setLocation(frontCratersLocationOnField);
//
//
//        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
//                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
//        backSpace.setLocation(backSpaceLocationOnField);
//
//
//        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
//        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
//        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
//
//        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
//                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));
//
//        /**  Let all the trackable listeners know where the phone is.  */
//        for (VuforiaTrackable trackable : allTrackablesNav) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        }
//
//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start tracking");
//        telemetry.update();
//
//
//        /** Start tracking the data sets we care about. */
//        targetsRoverRuckus.activate();
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addLine("op started");
            telemetry.update();
            auto.setMotorPower(new double[][]{{power, -power}, {power, -power}});

            // telemetry.addData("ad",robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
            //   AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            float[] positions = auto.getPositions();
            auto.scaledTurnImage(35, 0.3);
            if (auto.getPositions() == null)
                auto.searchImage();
            auto.driveToImage();
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


}

