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

        waitForStart();
        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
            LandInAuto(robot.hangingLockPosition,0.5);
            shaftGoDown(0.5, robot.shaftDownPosition);

              TensorflowUtils.GOLD_MINERAL_POSITION cubePosition= TensorflowUtils.GOLD_MINERAL_POSITION.RIGHT;


            telemetry.addData("Gold mineral position: ", cubePosition);
            telemetry.update();

            tsSampling.rotateToCube(0.5,robot.SamplingAngleRight,robot.SamplingAngleLeft,cubePosition);
            telemetry.addLine("press a for driveEncoderSamplingForward");
            telemetry.update();
//            while (!gamepad1.a);
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
            if(TensorflowUtils.isWebcamActivate) {
                vuforia.close();
                tsSampling.initVuforia(false);
            }
            telemetry.addLine("finished driving into cube");
            telemetry.update();
            sleep(200);


            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingBackward, robot.driveEncoderSamplingBackward, -robot.powerEncoder, false);
            telemetry.addLine("finished driving out of cube");
            telemetry.update();

            sleep(200);


            driveUtils.Turn(robot.angleTurnToImage, 0.5);
            sleep(300);
            targetNav.startTracking();
            sleep(500);

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

           targetNav.driveToImage(-0.3);
            sleep(500);
            driveUtils.driveByEncoderRoverRuckus(robot.distToDepot, robot.distToDepot, -robot.powerEncoder, false);//to depot
//            Marker(0.5,robot.shaftTargetPositionMarker);  //marker
//            // driveUtils.driveByEncoderRoverRuckus(90, 90, -0.5, false);//to crater
//            driveUtils.driveByEncoderRoverRuckus(robot.distToCrater, robot.distToCrater, -robot.powerEncoder, false);//to crater
        }
    }
}