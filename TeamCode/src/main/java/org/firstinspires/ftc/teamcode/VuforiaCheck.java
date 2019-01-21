package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

@Autonomous(name = "VuforiaCheck")
public class VuforiaCheck extends AutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        targetNav = new ImageTargets(this);
        driveUtils = new DriveUtilities(this);
        tsSampling = new TensorflowUtils(this);
        tsSampling.initVuforiaWebCam(false);
        targetNav.startTracking();
//        tsSampling.initVuforiaWebCam(true);
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            tsSampling.initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//            telemetry.update();
//        }
//        tfod.activate();
//        //motorLock();
        waitForStart();
        //LandInAuto();
double power=0.3;
        int distLeft = 10;
        int distRight = 10;
        double angle=280;
        while (opModeIsActive()) {
//            telemetry.addData("pos",targetNav.getPositions());
//            telemetry.addData("left: ", distLeft);
//            telemetry.addData("right: ", distRight);
//            telemetry.addData("pow: ", power);
            float[] pos=targetNav.getPositions();
            telemetry.addData("pos: ",targetNav.getPositions()==null);
            if(pos!=null){
                telemetry.addData("x: ",pos[0]);
                telemetry.addData("Heading: ",pos[5]);

            }
            telemetry.addData("ANGLE: ",angle);
            telemetry.update();
            if (gamepad1.dpad_up) {

                distLeft += 1;
                distRight += 1;
                sleep(40);
                angle+=5;
            }
            if (gamepad1.dpad_down) {
                distLeft -= 1;
                distRight -= 1;
                sleep(40);
                angle-=5;
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
            if(gamepad1.y)
                power*=-1;
            if (gamepad1.dpad_left) {
                distLeft -= 1;
                sleep(40);
            }
            if (gamepad1.a)
                driveUtils.scaledTurn(angle,0.35);
//                driveUtils.driveByEncoderRoverRuckus(distRight, distLeft, power, false);
            if(gamepad1.left_bumper)
                          targetNav.searchImage(2, -0.23);


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

