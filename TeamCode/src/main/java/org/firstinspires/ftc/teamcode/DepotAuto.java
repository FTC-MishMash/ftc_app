package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name = "Depot")
public class DepotAuto extends AutoMode {


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

            TensorflowUtils.MINERAL_POSITION result = tsSampling.goldPosition();

            if (result != TensorflowUtils.MINERAL_POSITION.NONE) {
                goldPos = result;

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
            driveUtils.rotateToDepot(37,322,);

            telemetry.addLine("finished driving into cube");
            telemetry.update();
            driveUtils.Turn(robot.angleTurnToImage);

            MarkerWithIntake(1, 2000);
            driveUtils.driveByEncoderRoverRuckus(robot.distToImageBeforeCrater, robot.distToImageBeforeCrater, -robot.powerEncoder, false);//to crater
            driveUtils.diffTurn(robot.newAngleToDepot);//intake to carter
            driveUtils.driveByEncoderRoverRuckus(robot.distFromImageToCrater, robot.distFromImageToCrater, robot.powerEncoder, false);//to crater
            Parking(robot.shaftEncoderPositionPARKING, 1, robot.linearOpenPosition, robot.linearEncoderOutLock, 1);
        }
    }
}