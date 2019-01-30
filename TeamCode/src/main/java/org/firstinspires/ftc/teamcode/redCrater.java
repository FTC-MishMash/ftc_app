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
//test2
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
        telemetry.addLine("wait for start");
        telemetry.update();
        //servoLock(0.2);
        waitForStart();
//          LandInAuto(0);



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
            driveUtils.driveByEncoderRoverRuckus(robot.driveEncoderSamplingForward, robot.driveEncoderSamplingForward , 0.4, false);
            if (cubePosition!=1)
            {
                driveUtils.driveByEncoderRoverRuckus(30, 30, 0.36, false);
                driveUtils.driveByEncoderRoverRuckus(-30, -30, -0.36, false);
            }
            else
            {
                driveUtils.driveByEncoderRoverRuckus(10, 10, 0.36, false);
                driveUtils.driveByEncoderRoverRuckus(-10, -10, -0.36, false);
            }
            if (tfod != null) {
                tfod.shutdown();
            }
            vuforia.close();
            telemetry.addLine("finished following");
            telemetry.update();
            sleep(200);
            driveUtils.driveByEncoderRoverRuckus(28, 28, 0.4, false);
            telemetry.addLine("finished driving into cube");
            telemetry.update();
            sleep(500);
            driveUtils.driveByEncoderRoverRuckus(-36, -36, -0.4, false);
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


            sleep(1000);
            targetNav.driveToImage(-0.25);

        }
    }
}
