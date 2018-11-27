package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by user on 22/11/2018.
 */

public class autoMode extends LinearOpMode {
    //    private static final android.graphics.Color Color = ;
    Robot robot;
    DcMotor[][] motors;
    ElapsedTime runTime = new ElapsedTime();
    final double SCALE_FACTOR = 255;

    static final int PitchtargetAngleMin = -5;
    static final int PitchtargetAngleMax = 5;
    static final int RolltargetAngleMin = -10;
    static final int RolltargetAngleMax = 10;



    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        waitForStart();
        runTime.startTime();
        runTime.reset();
        motors = robot.getDriveTrain();
        if (opModeIsActive()) {
            getDown dow = new getDown();
            //  TODO: add the funcition from the class

        }
    }

    public void getOffTheClimb(BNO055IMU imu, DcMotor[] motorsHanging, double power) {
        Driving.set2MotorPower(motorsHanging, power);
        while (!straightToField(imu)) ;
        Driving.set2MotorPower(motorsHanging, 0);
    }

    public  boolean straightToField(BNO055IMU imu) {
        Orientation axis = getAxis(imu);
        return axis.secondAngle > RolltargetAngleMin && axis.secondAngle < RolltargetAngleMax && axis.thirdAngle > PitchtargetAngleMin && axis.thirdAngle < PitchtargetAngleMax;

    }

    public Orientation getAxis(BNO055IMU imu) {
        return imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void ResetHue(ColorSensor color, float[] hsvArr) { //Reset the sensor color to read bt hue values.
        Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                (int) (color.green() * SCALE_FACTOR),
                (int) (color.blue() * SCALE_FACTOR),
                hsvArr);
    }

    public void setMotorPower(double[][] power) { //Stores the four drivetrain motors power in array
        for (int row = 0; opModeIsActive() && row < 2; row++)
            for (int col = 0; opModeIsActive() && col < 2; col++)
                robot.driveTrain[row][col].setPower(power[row][col]);
    }

    public void straightOnLine(int color, double power) {

        ResetHue(robot.colorRightFront,robot.valuesRightFront);
        ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
        telemetry.addData("hsvValues[0]", robot.hsvValuesRightFront[0]);
        telemetry.update();
        if (color == 0) {

            double time = getRuntime();
            while (opModeIsActive() && robot.hsvValuesRightFront[0] > robot.redColorRightSensor && robot.valuesLeftFront[0] > robot.redColorLeftSensor && (time + 1.5 > getRuntime())) {
                ResetHue(robot.colorRightFront,robot.valuesRightFront);
                ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
                setMotorPower(new double[][]{{power, power}, {power, power}});
                telemetry.addLine("search the First Line");
                telemetry.addData("hsvValuesRightFront[0]", robot.hsvValuesRightFront[0]);
                telemetry.update();
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});

            ResetHue(robot.colorRightFront,robot.valuesRightFront);
            ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
            if (robot.valuesRightFront[0] < robot.redColorRightSensor) {
                while (robot.valuesLeftFront[0] > robot.redColorLeftSensor && opModeIsActive()) {
                    setMotorPower(new double[][]{{0.75 * power, 0}, {0.75 * power, 0}});
                    telemetry.addData("search the left Line", robot.valuesLeftFront[0]);
                    telemetry.update();
                }
            }
            if (robot.valuesLeftFront[0] < robot.redColorLeftSensor) {
                while (robot.valuesRightFront[0] > robot.redColorRightSensor && opModeIsActive()) {
                    setMotorPower(new double[][]{{0, 0.75 * power}, {0, 0.75 * power}});
                    telemetry.addData("search the Right Line", robot.valuesRightFront[0]);
                    telemetry.update();
                }
            }

        }
        if (color == 1) {

            double time = getRuntime();
            while (opModeIsActive() && robot.hsvValuesRightFront[0] > robot.blueColorRightSensor && robot.valuesLeftFront[0] > robot.blueColorLeftSensor && (time + 1.5 > getRuntime())) {
                ResetHue(robot.colorRightFront,robot.valuesRightFront);
                ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
                setMotorPower(new double[][]{{power, power}, {power, power}});
                telemetry.addLine("search the First Line");
                telemetry.addData("hsvValuesRightFront[0]", robot.hsvValuesRightFront[0]);
                telemetry.update();
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});

            ResetHue(robot.colorRightFront,robot.valuesRightFront);
            ResetHue(robot.colorLeftFront, robot.valuesLeftFront);
            if (robot.valuesRightFront[0] < robot.redColorRightSensor) {
                while (robot.valuesLeftFront[0] > robot.redColorLeftSensor && opModeIsActive()) {
                    setMotorPower(new double[][]{{0.75 * power, 0}, {0.75 * power, 0}});
                    telemetry.addData("search the left Line", robot.valuesLeftFront[0]);
                    telemetry.update();
                }
            }
            if (robot.valuesLeftFront[0] < robot.redColorLeftSensor) {
                while (robot.valuesRightFront[0] > robot.redColorRightSensor && opModeIsActive()) {
                    setMotorPower(new double[][]{{0, 0.75 * power}, {0, 0.75 * power}});
                    telemetry.addData("search the Right Line", robot.valuesRightFront[0]);
                    telemetry.update();
                }
            }

        }
    }
}