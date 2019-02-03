package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name = "only sampling")
public class onlySampling extends AutoMode {

//    autoMode auto;

    @Override
    public void runOpMode() throws InterruptedException {
//        auto = new autoMode();
        super.runOpMode();
//        robot = new Robot(hardwareMap);
//
//        initVuforiaWebCam(hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

//        motorLock();
        waitForStart();
        LandInAuto(robot.hangingLockPosition,0.5);
        shaftGoDown(0.5, 2700);


        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();

            int cubePosition = 0;
            cubePosition = tsSampling.searchCube(0.33, robot.SamplingAngleRight, robot.SamplingAngleLeft);

            telemetry.addData("Gold mineral position: ", cubePosition);
            telemetry.update();
            if (tfod != null)
                tfod.activate();
//            sleep(800);
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingForward, robot.driveEncoderSamplingForward, robot.powerEncoder, false);
            if (cubePosition != 1) {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSide, robot.driveEncoderSamplingPositionSide, robot.powerEncoder, false);
            } else {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionMiddle, robot.driveEncoderSamplingPositionMiddle, robot.powerEncoder, false);
            }
            if (tfod != null) {
                tfod.shutdown();
            }


            Parking(1000,0.5,-750,0.5);
//            driveByEncoderRoverRuckus(-20, -20, 0.5);
//            setMotorPower(new double[][]{{-0.4, -0.4}, {-0.4, -0.4}});
//            sleep(500);
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});


        }
    }
}

