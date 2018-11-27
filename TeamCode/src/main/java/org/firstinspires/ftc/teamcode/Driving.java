package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by user on 30/10/2018.
 */

public class Driving {
    public static void ScaledTurn(double goalAngle, DcMotor[][] driveMotors, BNO055IMU imu, double power, Telemetry telemetry) {
        boolean sideOfTurn = true;
        double deltaAngle = 0;
        boolean directTurn = true;
        double currentAngle = getCurrentScaledAngle(imu);
        double angle0 = currentAngle;
        if (currentAngle < goalAngle) {
            if (goalAngle - currentAngle <= 360 - (goalAngle - currentAngle)) {
                sideOfTurn = false;
                deltaAngle = goalAngle - currentAngle;
            } else {
                sideOfTurn = true;
                deltaAngle = 360 - (goalAngle - currentAngle);
                directTurn = false;
            }


        } else {
            if (currentAngle - goalAngle <= 360 - (currentAngle - goalAngle)) {
                sideOfTurn = true;
                deltaAngle = currentAngle - goalAngle;
            } else {
                sideOfTurn = false;
                deltaAngle = 360 - (currentAngle - goalAngle);
                directTurn = false;
            }
        }
        if (sideOfTurn)
            setMotorPower(driveMotors, new double[][]{{power, -power}, {power, -power}});
        else
            setMotorPower(driveMotors, new double[][]{{-power, power}, {-power, power}});
        if (directTurn)
            while (Math.abs(angle0 - currentAngle) < deltaAngle) {  //motors running
                currentAngle = getCurrentScaledAngle(imu);
                telemetry.addData("angle case 3:", currentAngle);
                telemetry.update();
            }
        else if (goalAngle > 180 && currentAngle < 180)
            while ((currentAngle <= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle > 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                currentAngle = getCurrentScaledAngle(imu);
                telemetry.addData("angle case 1:", currentAngle);
                telemetry.update();
            }

        else if (goalAngle < 180 && currentAngle > 180)
            while ((currentAngle >= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle < 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                currentAngle = getCurrentScaledAngle(imu);
                telemetry.addData("angle case 2:", currentAngle);
                telemetry.update();
            }


        setMotorPower(driveMotors, new double[][]{{0, 0}, {0, 0}});
    }

    public static void setMotorPower(DcMotor[][] motors, double[][] powers) {
        for (int i = 0; i < motors.length; i++)
            for (int j = 0; j < motors[i].length; j++)
                motors[i][j].setPower(powers[i][j]);
    }

    public static void set2MotorPower(DcMotor[] motors, double powers) {
        for (int i = 0; i < motors.length; i++)
            motors[i].setPower(powers);
    }

    public static double getCurrentScaledAngle(BNO055IMU imu) {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle < 0)
            angle += 360;
        return angle;
    }

}

