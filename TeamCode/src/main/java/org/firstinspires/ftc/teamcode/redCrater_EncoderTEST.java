package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name = "cretar - ENCODER TESTING")
public class redCrater_EncoderTEST extends AutoMode {

//    autoMode auto;

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
        waitForStart();

        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
            LandInAuto(0.5);
            shaftGoDown(0.4,-250);

            int cubePosition = 0;
            //bePosition = tsSampling.searchCube(0.4, robot.SamplingAngleRight, robot.SamplingAngleLeft);

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
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingBackward, robot.driveEncoderSamplingBackward, -robot.powerEncoder, false);
            telemetry.addLine("finished driving out of cube");


            telemetry.update();
//            sleep(500);




            driveUtils.Turn(robot.newAngleTurnToImage);
//            sleep(2500);


            //driveUtils.back_up_driveByImage(0.45, robot.AngleToDepot, -(30 + cubePosition * 15));
            driveUtils.back_up_driveByImage(0.45, robot.newAngleToDepot, 160);
            //driveUtils.driveByEncoderRoverRuckus(-80, -80, -0.4, false);//to depot
            driveUtils.driveByEncoderRoverRuckus(robot.distToDepot, robot.distToDepot, robot.powerEncoder, false);//to depot
            Marker(0.5,robot.shaftTargetPositionMarker);  //marker
//            driveUtils.driveByEncoderRoverRuckus(-robot.distToCrater/2, -robot.distToCrater/2, -robot.powerEncoder, false);//to crater
            driveUtils.Turn(10);
            driveUtils.Turn(330);
//            driveUtils.driveByEncoderRoverRuckus(robot.distToCrater/2, robot.distToCrater/2, robot.powerEncoder, false);//to crater
            Parking(robot.shaftEncoderPositionPARKING,0.5,robot.linearOpenPosition,-70,0.3);
            //TODO: to change linear target encoder!!
//            driveByColor(0,robot.colorRightFront,robot.imu,robot.hsvValuesRightFront,AngleToDepot,0.35);
//            }
//            sleep(1000);
            //       driveUtils.driveByEncoderRoverRuckus(60, 60, 0.5,false);

            //go to crater
            //open shaft

        }
    }
}
