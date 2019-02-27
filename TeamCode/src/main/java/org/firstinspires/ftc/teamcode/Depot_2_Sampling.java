package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name = "Depot 2 Sampling")
public class Depot_2_Sampling extends AutoMode {


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
        telemetry.addData("Marker!!   ", "niv the boss");
        while (!isStarted()) {

            TensorflowUtils.MINERAL_POSITION result = tsSampling.goldPosition();

            if (result != TensorflowUtils.MINERAL_POSITION.NONE) {
                goldPos = result;
                telemetry.addData("Marker!!   ", "niv the boss");
                telemetry.addData("pos: ", goldPos);
                telemetry.update();
            }
        }
        if (goldPos == TensorflowUtils.MINERAL_POSITION.NONE)
            goldPos = TensorflowUtils.MINERAL_POSITION.LEFT;
        if (tfod != null) {
            tfod.shutdown();
        }
        waitForStart();
        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
            LandInAuto();
            shaftGoDown(0.8, 0);


            telemetry.addData("Gold mineral position: ", goldPos);
            telemetry.update();
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderAfHanging, robot.driveEncoderAfHanging, robot.powerEncoder, false);

            tsSampling.rotateToCube(robot.SamplingAngleRight, robot.SamplingAngleLeft, goldPos);
            telemetry.addLine("press a for driveEncoderSamplingForward");
            telemetry.update();

            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingForward, robot.driveEncoderSamplingForward, robot.powerEncoder, false);
            if (goldPos != TensorflowUtils.MINERAL_POSITION.CENTER) {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSide, robot.driveEncoderSamplingPositionSide, robot.powerEncoder, false);
            } else {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionMiddle, robot.driveEncoderSamplingPositionMiddle, robot.powerEncoder, false);
            }
            driveUtils.rotateToDepot(30, 335, goldPos);
            driveUtils.driveByEncoderRoverRuckus(robot.distFromSamplingToDepot, robot.distFromSamplingToDepot, robot.powerEncoder, false);
            MarkerWithIntake(robot.markerIntakePower, 2000);
//            driveUtils.driveByEncoderRoverRuckus(robot.distAfterMarkerToWall, robot.distAfterMarkerToWall, robot.powerEncoder, false);
            driveUtils.Turn(robot.angleAfterMarkerToWallDepot_2SAM);
            if (goldPos == TensorflowUtils.MINERAL_POSITION.LEFT)
                driveUtils.driveByEncoderRoverRuckus(robot.RIGHTdistAfterMarker_toWall_crater, robot.RIGHTdistAfterMarker_toWall_crater, robot.powerEncoder, false);
            else if (goldPos == TensorflowUtils.MINERAL_POSITION.CENTER)
                driveUtils.driveByEncoderRoverRuckus(robot.CENTERdistAfterMarker_toWall_crater, robot.CENTERdistAfterMarker_toWall_crater, robot.powerEncoder, false);
            else
                driveUtils.driveByEncoderRoverRuckus(robot.LeftdistAfterMarker_toWall_crater, robot.LeftdistAfterMarker_toWall_crater, robot.powerEncoder, false);

            driveUtils.Turn(robot.angleDepotToCrater_2SAM);

            if (goldPos == TensorflowUtils.MINERAL_POSITION.LEFT)
                driveUtils.driveByEncoderRoverRuckus(robot.RightDist_afterSampling_Depot, robot.RightDist_afterSampling_Depot, robot.powerEncoder, false);
            else if (goldPos == TensorflowUtils.MINERAL_POSITION.CENTER)
                driveUtils.driveByEncoderRoverRuckus(robot.CenterDist_afterSampling_Depot, robot.CenterDist_afterSampling_Depot, robot.powerEncoder, false);

            driveUtils.driveByEncoderRoverRuckus(robot.distFromDepotToCrater, robot.distFromDepotToCrater, robot.powerEncoder, false);//to crater
            driveUtils.Turn(180);
            Sampling_secondTime(goldPos,robot.linearEncoderFirst,robot.linearEncoderSecond,robot.linearEncoderThird,
                    robot.shaftEncoderSampling_2SAM,robot.linearEncoderMOVE_intake,robot.linearEncoderOutLock,1,0.6);
            driveUtils.Turn(robot.angleToCrater_2SAM);
//            Parking(robot.shaftEncoderPositionPARKING, 1, robot.linearOpenPosition, robot.linearEncoderOutLock, 1);
        }

    }
}