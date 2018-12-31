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


        waitForStart();

        runTime.reset();
        runTime.startTime();
        if (opModeIsActive()) {

//        getOffTheClimb

            int cubePosition = 0;
            cubePosition = searchCube(0.35, 345, 15, robot.driveTrain, robot.imu);

            if (tfod != null) {
                tfod.activate();
            }
            sleep(1000);
            followCubeRecognision(0.15);//start power

            if (tfod != null) {
                tfod.shutdown();
            }
            driveByEncoderRoverRuckus(7, 7, 0.5);
            sleep(2000);
            driveByEncoderRoverRuckus(-20, -20, 0.5);
            sleep(2000);
            ScaledTurn(60, robot.driveTrain, robot.imu, 0.5);

            //צריך להשתמש בcubePosition
            //פונקציות של מור
            startTracking(hardwareMap);
            float[] pos = getPositions();
            if (pos == null)
                searchImage(cubePosition);
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            pos = getPositions();
            telemetry.addData("pos", pos == null);
            telemetry.update();
            sleep(3000);
            driveToImage();
//            driveByColor(0, robot.colorRightFront, robot.imu, robot.hsvValuesRightFront, 135, 0.4);

        }
    }
}

