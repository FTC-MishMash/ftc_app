package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
@Autonomous(name = "red cretar")
public class redCrater extends LinearOpMode {
    Robot robot;
    autoMode auto;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        auto.initVuforiaWebCam();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            auto.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        if (tfod != null) {
            tfod.activate();
        }
        int cubePlace = -1;//dont see any cube.
        while (!isStarted()) {
            cubePlace = auto.getCube();//update cube location
            sleep(2);
        }
        if (tfod != null) {
            tfod.deactivate();
        }

        waitForStart();
//        int cubePlace = -1;//dont see any cube
//            cubePlace = getCube();//update cube location
        auto.runTime.reset();
        auto.runTime.startTime();
        if (opModeIsActive()) {

//        getOffTheClimb(robot.imu, robot.shaft, 0.3);

            if (cubePlace == -1) {//there is NOT cube/ or only one ball

            } else if (cubePlace == 0) {//see only 2 balls

                auto.ScaledTurn(50, robot.driveTrain, robot.imu, 0.5);

            } else if (cubePlace == 1) {//cube RIGHT

                auto.ScaledTurn(15, robot.driveTrain, robot.imu, 0.5);

            } else if (cubePlace == 2) {//cube LEFT

                auto.ScaledTurn(70, robot.driveTrain, robot.imu, 0.5);

            } else if (cubePlace == 3) {//cube CENTER
                //No need to move


            } else if (cubePlace == 4) {//cube RIGHT in camera
//TODO: add what the robot need to do in this
//            Driving.ScaledTurn(50,motors,robot.imu,0.5,telemetry);

            } else if (cubePlace == 5) {//cube LEFT in camera
//TODO: add what the robot need to do in this
//            Driving.ScaledTurn(50,motors,robot.imu,0.5,telemetry);

            }
            if (tfod != null) {
                tfod.activate();
            }
            auto.followCubeRecognision(0.15);//start power
            if (tfod != null) {
                tfod.shutdown();
            }
            auto.driveByEncoderRoverRuckus(7, 7, 0.5);
            sleep(2000);
            auto.driveByEncoderRoverRuckus(-20, -20, 0.5);
            sleep(2000);
            auto.ScaledTurn(60, robot.driveTrain, robot.imu, 0.5);
            //פונקציות של מור
            auto.driveByColor(0,robot.colorRightFront,robot.imu,robot.hsvValuesRightFront,135,0.4);

        }
    }
}
