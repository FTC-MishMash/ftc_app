package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red cretar")
public class redCrater extends LinearOpMode {
    Robot robot;
    autoMode auto;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        auto = new autoMode();
        auto.initVuforiaWebCam();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            auto.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

//        if (auto.tfod != null) {
//            auto.tfod.activate();
//        }
//        int cubePlace = -1;//dont see any cube.
//        while (!isStarted()) {
//            cubePlace = auto.getCube();//update cube location
//            sleep(100);
//        }
//        if (auto.tfod != null) {
//            auto.tfod.deactivate();
//        }

        waitForStart();

        auto.runTime.reset();
        auto.runTime.startTime();
        if (opModeIsActive()) {

//        getOffTheClimb(robot.imu, robot.shaft, 0.3);

            int cubePosition = 0;
            cubePosition = auto.searchCube(0.3, 15, 50, robot.driveTrain, robot.imu);
//            if (cubePlace == -1) {//there is NOT cube/ or only one ball
//
//            } else if (cubePlace == 0) {//see only 2 balls
//
//                auto.ScaledTurn(50, robot.driveTrain, robot.imu, 0.5);
//
//            } else if (cubePlace == 1) {//cube RIGHT
//
//                auto.ScaledTurn(15, robot.driveTrain, robot.imu, 0.5);
//
//            } else if (cubePlace == 2) {//cube LEFT
//
//                auto.ScaledTurn(70, robot.driveTrain, robot.imu, 0.5);
//
//            } else if (cubePlace == 3) {//cube CENTER
//                //No need to turn
//
//
//            } else if (cubePlace == 4) {//cube RIGHT in camera
////TODO: add what the robot need to do in this
//
//            } else if (cubePlace == 5) {//cube LEFT in camera
////TODO: add what the robot need to do in this

        }
        if (auto.tfod != null) {
            auto.tfod.activate();
        }
        auto.followCubeRecognision(0.15);//start power

        if (auto.tfod != null) {
            auto.tfod.shutdown();
        }
        auto.driveByEncoderRoverRuckus(7, 7, 0.5);
        sleep(2000);
        auto.driveByEncoderRoverRuckus(-20, -20, 0.5);
        sleep(2000);
        auto.ScaledTurn(60, robot.driveTrain, robot.imu, 0.5);
        //צריך להשתמש בcubePosition
        //פונקציות של מור
        auto.driveByColor(0, robot.colorRightFront, robot.imu, robot.hsvValuesRightFront, 135, 0.4);

    }
    public void initVuforiaWebCam() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = auto.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        auto.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        auto.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, auto.vuforia);
        auto.tfod.loadModelFromAsset(auto.TFOD_MODEL_ASSET, auto.LABEL_GOLD_MINERAL, auto.LABEL_SILVER_MINERAL);
    }
}

