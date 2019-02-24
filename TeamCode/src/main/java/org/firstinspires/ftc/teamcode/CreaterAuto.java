package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name = "Creater")
public class CreaterAuto extends AutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tsSampling.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }
        if (tfod != null)
            tfod.activate();
        robot.hanging.setPosition(robot.hangingLockPosition);
        telemetry.addLine("wait for start");
        telemetry.update();

        telemetry.addData("pos: ", goldPos);
        telemetry.update();
        while (!isStarted()) {
            TensorflowUtils.GOLD_MINERAL_POSITION result = tsSampling.goldPosition();
            if (result != TensorflowUtils.GOLD_MINERAL_POSITION.NONE) {
                goldPos = result;
                telemetry.addData("pos: ", goldPos);
                telemetry.update();
            }
        }
        if (goldPos == TensorflowUtils.GOLD_MINERAL_POSITION.NONE)
            goldPos = TensorflowUtils.GOLD_MINERAL_POSITION.LEFT;
        waitForStart();
        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
            // LandInAuto(robot.hangingLockPosition,0.5);
            //shaftGoDown(0.5, robot.shaftDownPosition);

            TensorflowUtils.GOLD_MINERAL_POSITION cubePosition = TensorflowUtils.GOLD_MINERAL_POSITION.LEFT;


            telemetry.addData("Gold mineral position: ", cubePosition);
            telemetry.update();
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderAfHanging, robot.driveEncoderAfHanging, robot.powerEncoder, false);

            tsSampling.rotateToCube(0.5, robot.SamplingAngleRight, robot.SamplingAngleLeft, cubePosition);
            telemetry.addLine("press a for driveEncoderSamplingForward");
            telemetry.update();

            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingForward, robot.driveEncoderSamplingForward, robot.powerEncoder, false);
            if (cubePosition != TensorflowUtils.GOLD_MINERAL_POSITION.CENTER) {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSide, robot.driveEncoderSamplingPositionSide, robot.powerEncoder, false);
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSideBackward, robot.driveEncoderSamplingPositionSideBackward, -robot.powerEncoder, false);
            } else {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionMiddle, robot.driveEncoderSamplingPositionMiddle, robot.powerEncoder, false);
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionMiddleBackward, robot.driveEncoderSamplingPositionMiddleBackward, -robot.powerEncoder, false);
            }
            if (tfod != null) {
                tfod.shutdown();
            }
            if (TensorflowUtils.isWebcamActivate) {
                vuforia.close();
                tsSampling.initVuforia(false);
            }
            telemetry.addLine("finished driving into cube");
            telemetry.update();


            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingBackward, robot.driveEncoderSamplingBackward, -robot.powerEncoder, false);
            telemetry.addLine("finished driving out of cube");
            telemetry.update();

//            sleep(200);


            driveUtils.Turn(robot.angleTurnToImage);
//            sleep(400);
            targetNav.startTracking();
            sleep(200);

            float[] pos = targetNav.getPositions();
            telemetry.addData("pos null: ", pos == null);
            telemetry.update();
            sleep(200);
            if (pos == null) {

                telemetry.addData("start searching wait for click", cubePosition);
                telemetry.update();
                sleep(300);
                targetNav.searchImage(cubePosition, -0.20);
            }
            pos = targetNav.getPositions();
            if (pos != null) {
                targetNav.driveToImage(-0.28);
            } else {
                telemetry.addLine("no image");
                driveUtils.driveByEncoderRoverRuckus(-57, -57, -0.5, false);
                telemetry.addLine("finished alt encoders");
                driveUtils.Turn(127);
                telemetry.addLine("finished turnong");
                telemetry.update();
            }
            sleep(400);
            driveUtils.driveByEncoderRoverRuckus(robot.distToDepot, robot.distToDepot, robot.powerEncoder, false);//to depot
            //TODO: להפוך את הנסיעות
//            Marker(0.5, robot.shaftTargetPositionMarker);  //marker
            MarkerWithIntake(-1, 2000);
            driveUtils.driveByEncoderRoverRuckus(robot.distToImageBeforeCrater, robot.distToImageBeforeCrater, -robot.powerEncoder, false);//to crater
            driveUtils.diffTurn(robot.newAngleToDepot);//intake to carter
            driveUtils.driveByEncoderRoverRuckus(31, 31, 0.4, false);//to crater
            Parking(robot.shaftEncoderPositionPARKING, 0.4, robot.linearOpenPosition, 1);
        }
    }
}