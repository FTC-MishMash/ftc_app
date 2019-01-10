package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name = "Red cretar")
public class redCrater extends AutoMode {

//    autoMode auto;

    @Override
    public void runOpMode() throws InterruptedException {
//        auto = new autoMode();
        super.runOpMode();
        tsSampling.initVuforiaWebCam(true);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tsSampling.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        //motorLock();
        waitForStart();
        //LandInAuto();


        while (opModeIsActive()) {
            if(vuforia==null)
                telemetry.addLine("null");
            if(gamepad1.x)
            {
                vuforia.close();
            telemetry.addLine("closed");

            }
            if(gamepad1.a)
                tsSampling.initVuforiaWebCam(false);
            telemetry.update();
//            runTime.reset();
//            runTime.startTime();
////        getOffTheClimb
//
//            int cubePosition = 0;
//            cubePosition = tsSampling.searchCube(0.35, 335, 20);
//
//            sleep(1000);
//            tsSampling.followCubeRecognision(0.15);//start power
//
//            if (tfod != null) {
//                tfod.shutdown();
//            }
//
//            driveUtils.driveByEncoderRoverRuckus(15, 15, 0.5);
//            sleep(2500);
////            driveByEncoderRoverRuckus(-20, -20, 0.5);
//            DriveUtilities.setMotorPower(robot.driveTrain,new double[][]{{-0.4, -0.4}, {-0.4, -0.4}});
//            sleep(500);
//            DriveUtilities.setMotorPower(robot.driveTrain,new double[][]{{0, 0}, {0, 0}});
//            sleep(1500);
//            driveUtils.scaledTurn(60, 0.4);
//            sleep(1000);
//            targetNav.startTracking();
//            float[] pos = targetNav.getPositions();
//            if (pos == null)
//                targetNav.searchImage(cubePosition, 0.24);
//            targetNav.driveToImage(0.3);
//
//            pos = getPositions();//למה להשתמש בPOS ולא פשוט בפונקציה?
//            telemetry.addData("pos", pos == null);
//            telemetry.update();
//            if (pos == null) {
//                driveByEncoderRoverRuckus(20, 20, 0.4);
//                ScaledTurn(135, robot.driveTrain, robot.imu, 0.3);
//            } else {
//                sleep(1000);
//                driveToImage(-0.19);
//            }
//            sleep(1000);
//            driveByEncoderRoverRuckus(60, 60, 0.5);
            //marker
            //go to crater
            //open shaft

        }
    }
}

