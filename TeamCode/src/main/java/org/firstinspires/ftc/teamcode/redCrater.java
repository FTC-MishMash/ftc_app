package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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
        super.runOpMode();
//        auto = new autoMode();
        // robot = new Robot(hardwareMap);

        tsSampling.initVuforiaWebCam(true);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tsSampling.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        servoLock(1);
        waitForStart();
       // LandInAuto(0);


        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
//        getOffTheClimb

            int cubePosition = 0;
          //  cubePosition = tsSampling.searchCube(0.35, 335, 20);

            sleep(1000);
          //  tsSampling.followCubeRecognision(0.15);//start power

            if (tfod != null) {
                tfod.shutdown();
            }
            vuforia.close();
         //   driveUtils.driveByEncoderRoverRuckus(15, 15, 0.5);
            sleep(2500);
//            driveByEncoderRoverRuckus(-20, -20, 0.5);
            driveUtils.setMotorPower(robot.driveTrain, new double[][]{{-0.4, -0.4}, {-0.4, -0.4}});
            sleep(500);
            driveUtils.setMotorPower(robot.driveTrain, new double[][]{{0, 0}, {0, 0}});
            sleep(2500);
           // driveUtils.scaledTurn(60, 0.4);
            tsSampling.initVuforiaWebCam(false);
            sleep(1000);
//            צריך להשתמש בcubePosition
//            פונקציות של מור
            targetNav.vuforia=vuforia;
            targetNav.startTracking();
//            float[] pos = targetNav.getPositions();
//            if (pos == null)
//                targetNav.searchImage(cubePosition, 0.24);
//
//            pos = targetNav.getPositions();//למה להשתמש בPOS ולא פשוט בפונקציה?
//            telemetry.addData("pos", pos == null);
//            telemetry.update();
//            if (pos == null) {
//                driveUtils.driveByEncoderRoverRuckus(20, 20, 0.4);
//                driveUtils.scaledTurn(135, 0.3);
//            } else {
//                sleep(1000);
//                targetNav.driveToImage(-0.19);
//            }
//            sleep(1000);
            driveUtils.driveByEncoderRoverRuckus(60, 60, 0.5);
            //marker
            //go to crater
            //open shaft

        }
    }
}

