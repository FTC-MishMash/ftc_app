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

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tsSampling.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }
//        robot.hanging.setPosition(robot.hangingLockPosition);
        telemetry.addLine("wait for start");
        telemetry.update();
        waitForStart();
        LandInAuto(robot.hangingLockPosition,0.5);
        shaftGoDown(0.5, 2700);


        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
//        getOffTheClimb
//            driveUtils.TurnWithEncoder(50, 0.6);
//            sleep(1500);
//            driveUtils.TurnWithEncoder(0, 0.6);
//            sleep(1000);
            int cubePosition = 0;
            cubePosition = tsSampling.searchCube(0.33, robot.SamplingAngleRight, robot.SamplingAngleLeft);

            telemetry.addData("Gold mineral position: ", cubePosition);
            telemetry.update();
            if (tfod != null)
                tfod.activate();
            sleep(800);
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingForward, robot.driveEncoderSamplingForward, 0.4, false);
            if (cubePosition != 1) {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSide, robot.driveEncoderSamplingPositionSide, 0.36, false);
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionSideBackward, robot.driveEncoderSamplingPositionSideBackward, -0.36, false);
            } else {
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionMiddle, robot.driveEncoderSamplingPositionMiddle, 0.36, false);
                driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingPositionMiddleBackward, robot.driveEncoderSamplingPositionMiddleBackward, -0.36, false);
            }
            if (tfod != null) {
                tfod.shutdown();
            }
            vuforia.close();
            telemetry.addLine("finished driving into cube");
            telemetry.update();
            sleep(500);
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingBackward, robot.driveEncoderSamplingBackward, -0.4, false);
            telemetry.addLine("finished driving out of cube");
            telemetry.update();
            sleep(500);

            //  driveUtils.setMotorPower(robot.driveTrain, new double[][]{{0, 0}, {0, 0}});


            driveUtils.TurnWithEncoder(robot.angleTurnToImage, 0.5);
            sleep(500);
            tsSampling.initVuforiaWebCam(false);
            targetNav.startTracking();
            sleep(500);

            float[] pos = targetNav.getPositions();
            telemetry.addData("pos null: ", pos == null);
            telemetry.update();
            sleep(500);
            if (pos == null) {

                telemetry.addData("start searching wait for click", cubePosition);
                telemetry.update();
                sleep(500);
                targetNav.searchImage(cubePosition, -0.20);
            }


            sleep(300);
            targetNav.driveToImage(-0.25);
            sleep(200);
            driveUtils.driveByEncoderRoverRuckus(robot.distToDepot, robot.distToDepot, -0.5, false);//to depot
            Marker(0.5);  //marker
            // driveUtils.driveByEncoderRoverRuckus(90, 90, -0.5, false);//to crater
            driveUtils.driveByEncoderRoverRuckus(robot.distToCrater, robot.distToCrater, 0.5, false);//to crater
        }
    }
}
