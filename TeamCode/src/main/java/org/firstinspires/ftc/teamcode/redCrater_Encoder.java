package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name = "Red cretar - ENCODER")
public class redCrater_Encoder extends AutoMode {

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
        robot.hanging.setPosition(robot.hangingLockPosition);
        waitForStart();

        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
//            LandInAuto(robot.hangingOpenPosition);
//            shaftGoDown(0.4,-250);

            int cubePosition = 0;
            cubePosition = tsSampling.searchCube(0.33, robot.SamplingAngleRight, robot.SamplingAngleLeft);

            telemetry.addData("Gold mineral position: ", cubePosition);
            telemetry.update();
           // driveUtils.driveByEncoderRoverRuckus(20, 40, 0.35, false);

            telemetry.addLine("finished go to cube");
            telemetry.update();
            sleep(1500);
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingForward, robot.driveEncoderSamplingForward, 0.36, false);
            if (cubePosition!=1)
            {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSide, robot.driveEncoderSamplingPositionSide, 0.36, false);
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSideBackward,robot.driveEncoderSamplingPositionSideBackward, -0.36, false);
            }
            else
                {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionMiddle, robot.driveEncoderSamplingPositionMiddle, 0.36, false);
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionMiddleBackward, robot.driveEncoderSamplingPositionMiddleBackward, -0.36, false);
            }
            telemetry.addLine("finished driving into cube");
            telemetry.update();
            sleep(2500);
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingBackward, robot.driveEncoderSamplingBackward, -0.36, false);
            telemetry.addLine("finished driving out of cube");


            telemetry.update();
            sleep(500);




            driveUtils.TurnWithEncoder(robot.angleTurnToImage, 0.5);
            sleep(2500);


            //driveUtils.back_up_driveByImage(0.45, robot.AngleToDepot, -(30 + cubePosition * 15));
            driveUtils.back_up_driveByImage(0.45, robot.AngleToDepot, 95);
            //driveUtils.driveByEncoderRoverRuckus(-80, -80, -0.4, false);//to depot
            driveUtils.driveByEncoderRoverRuckus(robot.distToDepot, robot.distToDepot, -0.5, false);//to depot
            Marker(0.5);  //marker
           // driveUtils.driveByEncoderRoverRuckus(90, 90, -0.5, false);//to crater
            driveUtils.driveByEncoderRoverRuckus(180, 180, 0.5, false);//to crater

           // Parking(230,-3300,0.2,0,0);
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
