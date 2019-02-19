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

        telemetry.addLine("wait for start");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();

           // driveUtils.TurnWithEncoder(320, 0.35);//encoder=0.3
              TensorflowUtils.GOLD_MINERAL_POSITION cubePosition;

            cubePosition = TensorflowUtils.GOLD_MINERAL_POSITION.CENTER;

            telemetry.addData("Gold mineral position: ", cubePosition);
            telemetry.update();

            sleep(800);
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
            vuforia.close();
            telemetry.addLine("finished driving into cube");
            telemetry.update();
            sleep(300);


            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingBackward, robot.driveEncoderSamplingBackward, -robot.powerEncoder, false);
            telemetry.addLine("finished driving out of cube");
            telemetry.update();
            tsSampling.initVuforiaWebCam(false);
            sleep(200);

            //  driveUtils.setMotorPower(robot.driveTrain, new double[][]{{0, 0}, {0, 0}});

            driveUtils.Turn(robot.angleTurnToImage, 0.5);
            sleep(300);
            targetNav.startTracking();
            sleep(500);

            float[] pos = targetNav.getPositions();
            telemetry.addData("pos null: ", pos == null);
            telemetry.update();
//            sleep(500);
            if (pos == null) {

                telemetry.addData("start searching wait for click", cubePosition);
                telemetry.update();
               sleep(300);
                targetNav.searchImage(cubePosition, -0.20);
            }

            targetNav.driveToImage(-0.35);
            sleep(500);
            driveUtils.driveByEncoderRoverRuckus(robot.distToDepot, robot.distToDepot, -robot.powerEncoder, false);//to depot
//            Marker(0.5,robot.shaftTargetPositionMarker);  //marker
//            // driveUtils.driveByEncoderRoverRuckus(90, 90, -0.5, false);//to crater
//            driveUtils.driveByEncoderRoverRuckus(robot.distToCrater, robot.distToCrater, -robot.powerEncoder, false);//to crater
        }
    }
}