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
        telemetry.addData("Marker!!   ","niv the boss");
        while (!isStarted()) {

            TensorflowUtils.MINERAL_POSITION result = tsSampling.goldPosition();

            if (result != TensorflowUtils.MINERAL_POSITION.NONE) {
                goldPos = result;
                telemetry.addData("Marker!!   ","niv the boss");

                telemetry.addData("pos: ", goldPos);
                telemetry.update();
            }
        }
        if (goldPos == TensorflowUtils.MINERAL_POSITION.NONE)
            goldPos = TensorflowUtils.MINERAL_POSITION.LEFT;
        waitForStart();
        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
            LandInAuto();
            shaftGoDown(0.8, 0);



            telemetry.addData("Gold mineral position: ", goldPos);
            telemetry.update();
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderAfHanging, robot.driveEncoderAfHanging, robot.powerEncoder, false);

            tsSampling.rotateToCube( robot.SamplingAngleRight, robot.SamplingAngleLeft, goldPos);
            telemetry.addLine("press a for driveEncoderSamplingForward");
            telemetry.update();

            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingForward, robot.driveEncoderSamplingForward, robot.powerEncoder, false);
            if (goldPos != TensorflowUtils.MINERAL_POSITION.CENTER) {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSide, robot.driveEncoderSamplingPositionSide, robot.powerEncoder, false);
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSideBackward, robot.driveEncoderSamplingPositionSideBackward, -robot.powerEncoder, false);
            if(goldPos==TensorflowUtils.MINERAL_POSITION.RIGHT)
                driveUtils.driveByEncoderRoverRuckus(robot.rightSamplingBackward, robot.rightSamplingBackward, -robot.powerEncoder, false);
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


//            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingBackward, robot.driveEncoderSamplingBackward, -robot.powerEncoder, false);
            telemetry.addLine("finished driving out of cube");
            telemetry.update();

//            sleep(200);

            driveUtils.Turn(robot.angleTurnToImage);
//            sleep(400);
            targetNav.startTracking();
//            sleep(200);

            float[] pos = targetNav.getPositions();
            telemetry.addData("pos null: ", pos == null);
            telemetry.update();
//            sleep(200);
            if (pos == null) {

                targetNav.searchImage(goldPos, -0.35);
            }
            pos = targetNav.getPositions();
            if (pos != null) {
                targetNav.driveToImage(-robot.powerEncoder);//inbar change from 53 to 56
            } else {
                telemetry.addLine("no image");
                telemetry.update();
                driveUtils.driveByEncoderRoverRuckus(robot.encoderBACKUPtoImage, robot.encoderBACKUPtoImage, -robot.powerEncoder-0.15, false);
                telemetry.addLine("finished alt encoders");
                telemetry.update();
                driveUtils.Turn(robot.depotAngle);
                telemetry.addLine("finished turnong");
                telemetry.update();
            }
            driveUtils.driveByEncoderRoverRuckus(robot.distToDepot, robot.distToDepot, robot.powerEncoder, false);//to depot

            MarkerWithIntake(robot.markerIntakePower, 2000);
            driveUtils.driveByEncoderRoverRuckus(robot.distToImageBeforeCrater, robot.distToImageBeforeCrater, -robot.powerEncoder, false);//to crater
            driveUtils.diffTurn(robot.newAngleToDepot);//intake to carter
            driveUtils.driveByEncoderRoverRuckus(robot.distFromImageToCrater, robot.distFromImageToCrater, robot.powerEncoder, false);//to crater
            Parking(robot.shaftEncoderPositionPARKING, 1, robot.linearOpenPosition, robot.linearEncoderOutLock, 1);
        }
    }
}