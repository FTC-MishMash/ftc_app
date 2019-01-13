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
public class redCrater extends autoMode {

//    autoMode auto;

    @Override
    public void runOpMode() throws InterruptedException {
//        auto = new autoMode();
        robot = new Robot(hardwareMap);

        initVuforiaWebCam(hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        motorLock();
        waitForStart();
        LandInAuto();


        if (opModeIsActive()) {
            runTime.reset();
            runTime.startTime();
//        getOffTheClimb

            int cubePosition = 0;
            cubePosition = searchCube(0.35, 335, 20, robot.driveTrain, robot.imu);

            sleep(1000);
            followCubeRecognision(0.15);//start power

            if (tfod != null) {
                tfod.shutdown();
            }

            driveByEncoderRoverRuckus(15, 15, 0.5);
            sleep(2500);
//            driveByEncoderRoverRuckus(-20, -20, 0.5);
            setMotorPower(new double[][]{{-0.4,-0.4},{-0.4,-0.4}});
            sleep(500);
            setMotorPower(new double[][]{{0,0},{0,0}});
            sleep(2500);
            ScaledTurn(60, robot.driveTrain, robot.imu, 0.4);
            sleep(1000);
//            צריך להשתמש בcubePosition
//            פונקציות של מור
            startTracking();
            float[] pos = getPositions();
            if (pos == null)
                searchImage(cubePosition, 0.24);

            pos = getPositions();//למה להשתמש בPOS ולא פשוט בפונקציה?
            telemetry.addData("pos", pos == null);
            telemetry.update();
            if (pos == null) {
                driveByEncoderRoverRuckus(20, 20, 0.4);
                ScaledTurn(135, robot.driveTrain, robot.imu, 0.3);
            } else {
                sleep(1000);
                driveToImage(-0.19);
            }
            sleep(1000);
            driveByEncoderRoverRuckus(60, 60, 0.5);
            //marker
            //go to crater
            //open shaft

        }
    }
}

