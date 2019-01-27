package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.ClassFactoryImpl;

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
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tsSampling.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }
        servo();
        waitForStart();
        LandInAuto(0);


        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
//        getOffTheClimb

            int cubePosition = 0;
            cubePosition = tsSampling.searchCube(0.33, 335, 23);

            telemetry.addData("Gold mineral position: ", cubePosition);
            telemetry.update();
            if (tfod != null)
                tfod.activate();
            sleep(1200);
            tsSampling.followCubeRecognision(0.16);//start power

            if (tfod != null) {
                tfod.shutdown();
            }
            vuforia.close();
            telemetry.addLine("finished following");
            telemetry.update();
            sleep(1500);
            driveUtils.driveByEncoderRoverRuckus(14, 14, 0.36, false);
            telemetry.addLine("finished driving into cube");
            telemetry.update();
            sleep(2500);
            driveUtils.driveByEncoderRoverRuckus(-20, -20, -0.36, false);
            telemetry.addLine("finished driving out of cube");
            telemetry.update();
            sleep(2000);

            driveUtils.setMotorPower(robot.driveTrain, new double[][]{{0, 0}, {0, 0}});
            sleep(2500);
            double angleTurn = 265;
//            if(cubePosition==2)
//            angleTurn+=12;
//            else if(cubePosition==3)
//                angleTurn-=12;
            driveUtils.TurnWithEncoder(angleTurn, 0.4);
            sleep(2500);
            tsSampling.initVuforiaWebCam(false);
            targetNav.startTracking();
            sleep(1000);

            float[] pos = targetNav.getPositions();
            telemetry.addData("pos null: ", pos == null);
            telemetry.update();
            sleep(3500);
            if (pos == null) {

                telemetry.addData("start searching wait for click", cubePosition);
                telemetry.update();
                sleep(1200);
                targetNav.searchImage(cubePosition, -0.20);
            }

//            pos = targetNav.getPositions();//למה להשתמש בPOS ולא פשוט בפונקציה?
//            telemetry.addData("pos CHECKING!!0:", pos == null);
//            telemetry.update();
////            if (pos == null) {
////                driveUtils.driveByEncoderRoverRuckus(20, 20, 0.4);
////                driveUtils.scaledTurn(135, 0.3);
////            } else {
            sleep(1000);
            targetNav.driveToImage(-0.21);
//            }
//            sleep(1000);
            //       driveUtils.driveByEncoderRoverRuckus(60, 60, 0.5,false);
            //marker
            //go to crater
            //open shaft

        }
    }
}
