package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
@Autonomous(name = "AutoWithoutLanding")
public class MorAuto extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
//        auto = new autoMode();
        // robot = new Robot(hardwareMap);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tsSampling.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }
//        robot.hanging.setPosition(robot.hangingLockPosition);
        waitForStart();

        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();

            int cubePosition = 0;
            cubePosition = tsSampling.searchCube(0.35, robot.SamplingAngleRight, robot.SamplingAngleLeft);

            telemetry.addData("Gold mineral position: ", cubePosition);
            telemetry.update();
            // driveUtils.driveByEncoderRoverRuckus(20, 40, 0.35, false);

            telemetry.addLine("finished go to cube");
            telemetry.update();
//            sleep(1500);
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingForward, robot.driveEncoderSamplingForward, robot.powerEncoder, false);
            if (cubePosition!=1)
            {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSide, robot.driveEncoderSamplingPositionSide, robot.powerEncoder, false);
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSideBackward,robot.driveEncoderSamplingPositionSideBackward, -robot.powerEncoder, false);
            }
            else
            {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionMiddle, robot.driveEncoderSamplingPositionMiddle, robot.powerEncoder, false);
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionMiddleBackward, robot.driveEncoderSamplingPositionMiddleBackward, -robot.powerEncoder, false);
            }
            telemetry.addLine("finished driving into cube");
            telemetry.update();
//            sleep(2500);
            vuforia.close();
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingBackward, robot.driveEncoderSamplingBackward, -robot.powerEncoder, false);
            telemetry.addLine("finished driving out of cube");


            telemetry.update();
//            sleep(500);




            driveUtils.TurnWithEncoder(robot.angleTurnToImage, 0.5);
            sleep(100);
            tsSampling.initVuforiaWebCam(false);
            targetNav.startTracking();
//            sleep(500);

            float[] pos = targetNav.getPositions();
            telemetry.addData("pos null: ", pos == null);
            telemetry.update();
//            sleep(500);
            if (pos == null) {

                telemetry.addData("start searching wait for click", cubePosition);
                telemetry.update();
//                sleep(500);
                targetNav.searchImage(cubePosition, -0.20);
            }

//            sleep(600);
            targetNav.driveToImage(-0.3);
//            sleep(500);
            driveUtils.driveByEncoderRoverRuckus(robot.distToDepot, robot.distToDepot, robot.powerEncoder, false);//to depot
            Marker(0.5,robot.shaftTargetPositionMarker);  //marker
            // driveUtils.driveByEncoderRoverRuckus(90, 90, -0.5, false);//to crater
            driveUtils.driveByEncoderRoverRuckus(robot.distToCrater, robot.distToCrater, -robot.powerEncoder, false);//to crater

        }
    }
}