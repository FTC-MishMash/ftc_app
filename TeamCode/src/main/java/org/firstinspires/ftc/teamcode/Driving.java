package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by user on 30/10/2018.
 */
@Disabled
public class Driving {
    autoMode autoMode;
    static boolean sideOfTurn = true;
    static boolean directTurn = true;
    static double deltaAngle = 0;

    //TODO: change the set motor power (dc)


    public static double getCurrentScaledAngle(BNO055IMU imu) {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle < 0)
            angle += 360;
        return angle;
    }

    public static void setMotorPower(DcMotor[][] motors, double[][] powers) {
        for (int i = 0; i < motors.length; i++)
            for (int j = 0; j < motors[i].length; j++)
                motors[i][j].setPower(powers[i][j]);
    }

    static void setTurnDirection(double currentAngle, double goalAngle) {
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
    }
}
