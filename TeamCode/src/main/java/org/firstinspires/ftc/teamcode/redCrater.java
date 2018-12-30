package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red cretar")
public class redCrater extends autoMode {

//    autoMode auto;

    @Override
    public void runOpMode() throws InterruptedException {
//        auto = new autoMode();
        robot = new Robot(hardwareMap);

        initVuforiaWebCam(hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        waitForStart();

        runTime.reset();
        runTime.startTime();
        if (opModeIsActive()) {

//        getOffTheClimb

            int cubePosition = 0;
            cubePosition = searchCube(0.3, 15, 50, robot.driveTrain, robot.imu);

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
    }
}

