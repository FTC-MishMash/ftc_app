package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name = "only sampling")
public class onlySampling extends autoMode {

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

//        motorLock();
        waitForStart();
//        LandInAuto();


        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
//        getOffTheClimb

            int cubePosition = 0;
            cubePosition = searchCube(0.35, 335, 20, robot.driveTrain, robot.imu);

            sleep(1000);
            followCubeRecognision(0.15);//start power

            if (tfod != null) {
                tfod.shutdown();
            }

            driveByEncoderRoverRuckus(7, 7, 0.5);
            sleep(2500);
//            driveByEncoderRoverRuckus(-20, -20, 0.5);
            setMotorPower(new double[][]{{-0.4,-0.4},{-0.4,-0.4}});
            sleep(500);
            setMotorPower(new double[][]{{0,0},{0,0}});


        }
    }
}

