package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red cretar")
public class redCrater extends autoMode {
//    Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {
        auto = new autoMode();
        robot = new Robot(hardwareMap);

        initVuforiaWebCam();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
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

        runTime.reset();
        runTime.startTime();
        if (opModeIsActive()) {

//        getOffTheClimb(robot.imu, robot.shaft, 0.3);

            int cubePosition = 0;
            cubePosition = searchCube(0.3, 15, 50, robot.driveTrain, robot.imu);
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
        //צריך להשתמש בcubePosition
        //פונקציות של מור
        driveByColor(0, robot.colorRightFront, robot.imu, robot.hsvValuesRightFront, 135, 0.4);

    }
//    public void initVuforiaWebCam() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
//    }
//
//    /**
//     * Initialize the Tensor Flow Object Detection engine.
//     */
//    public void initTfod() {
//
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        auto.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, auto.vuforia);
//        auto.tfod.loadModelFromAsset(auto.TFOD_MODEL_ASSET, auto.LABEL_GOLD_MINERAL, auto.LABEL_SILVER_MINERAL);
//    }
}

