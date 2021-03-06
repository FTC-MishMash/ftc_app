package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name = "VuforiaCheck")
//@Disabled
public class VuforiaCheck extends AutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
           robot = new Robot(hardwareMap);
        targetNav = new ImageTargets(this);
         driveUtils = new DriveUtilities(this);
        tsSampling = new TensorflowUtils(this);

        tsSampling.initVuforia(false);
        // targetNav.startTracking();
//        tsSampling.initVuforiaWebCam(true);
//
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tsSampling.initTfod();
        }
        tfod.activate();
        int posCube = -1;
        while (!isStarted()) {

            TensorflowUtils.MINERAL_POSITION goldPos = tsSampling.goldPosition();
            if (goldPos != TensorflowUtils.MINERAL_POSITION.NONE) {
                telemetry.addData("pos: ", goldPos);
                telemetry.update();
            }
        }
        waitForStart();

        double power = 0.45;
        int motor = 0;
        int distLeft = 10;
        int distRight = 10;
        double angle = 280;
        while (opModeIsActive()) {
//            telemetry.addData("pos",targetNav.getPositions());
            telemetry.addData("left: ", distLeft);
//            telemetry.addData("right: ", distRight);
//            telemetry.addData("pow: ", power);

            telemetry.addData("motor pos: ", robot.driveTrain[0][0].getCurrentPosition());
            telemetry.addData("angle:   ", angle);
            telemetry.addData("imu", DriveUtilities.normalizedAngle(getAngularOriention().firstAngle));
            float[] pos = targetNav.getPositions();
//            telemetry.addData("pos: ",targetNav.getPositions()==null);
            if (pos != null) {
                telemetry.addData("x: ", pos[0]);
                telemetry.addData("Heading: ", pos[5]);

            }
//            telemetry.addData("ANGLE: ",angle);
            telemetry.update();
            if (gamepad1.left_stick_button) {
                driveUtils.Turn(angle);
            }

            if (gamepad1.dpad_up) {

                distLeft += 1;
                distRight += 1;
                sleep(40);
                angle += 5;
            }
            if (gamepad1.left_bumper) {
                motor++;
                sleep(70);
            }
            if (gamepad1.right_bumper) {
                motor--;
                sleep(70);
            }
            if (gamepad1.dpad_down) {
                distLeft -= 1;
                distRight -= 1;
                sleep(40);
                angle -= 5;
            }
            if (gamepad1.dpad_right) {
                distRight -= 1;
                sleep(40);
            }
            if (gamepad1.b) {
                distRight += 1;
                sleep(40);
            }
            if (gamepad1.x) {
                distLeft += 1;
                sleep(40);
            }
            if (gamepad1.y)
                power *= -1;
            if (gamepad1.dpad_left) {
                distLeft -= 1;
                sleep(40);
            }
            //   if (gamepad1.a)
            // driveUtils.DriveByDistance(distLeft,power,0);
            //    driveUtils.driveByEncoderRoverRuckus(distRight, distLeft, power, false);
//            if(gamepad1.left_bumper)
//                          targetNav.searchImage(2, -0.23);


//                tsSampling.followCubeRecognision(0.5);
//                telemetry.addLine("null");
//            if (gamepad1.x) {
//                tfod.deactivate();
//                vuforia.close();
////                VuforiaLocalizerImpl.CloseableFrame frame=vuforia.getFrameQueue().take();
////                frame.close();
//                telemetry.addLine("closed");
//
//            }
//            if (gamepad1.a){
//                tsSampling.initVuforiaWebCam(false);
//                tfod.activate();
//            }
//            telemetry.update();

        }
    }
}


