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
            while (
                    (currentAngle <= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle > 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
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
    public static void  driveByEncoderRoverRuckus(DcMotor[][] driveTrain,double goalDistRight, double goalDistLeft, double power) {// Drive by encoders and converts incoders ticks to distance in cm and drives until distance is completed.
        //Reset encoders

        final double tixRound = 600;
        final double cmRound = 27;

        double dRight = (goalDistRight * tixRound) / cmRound;
        double dLeft = (goalDistLeft * tixRound) / cmRound;

      driveTrain[0][0].setTargetPosition((int) (driveTrain[0][0].getCurrentPosition() + dLeft));
      driveTrain[1][0].setTargetPosition((int) (driveTrain[1][0].getCurrentPosition() + dLeft));

      driveTrain[0][1].setTargetPosition((int) (driveTrain[0][1].getCurrentPosition() + dRight));
      driveTrain[1][1].setTargetPosition((int) (driveTrain[1][1].getCurrentPosition() + dRight));


      driveTrain[0][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
      driveTrain[1][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
      driveTrain[0][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
      driveTrain[1][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
              driveTrain[i][j].setPower(power);


//        double err = 200;
//        while (err > 100 && opModeIsActive()) {
//            err = 0;
//            for (int i = 0; i < 2; i++)
//                for (int j = 0; j < 2; j++) {
//                    err += Math.abs(goalEncoder[i][j] - robot.driveTrain[i][j].getCurrentPosition());
//                    telemetry.addData(" encoder", robot.driveTrain[i][j].getCurrentPosition());
//                }
//            err /= 4;
//
//            telemetry.addData(" err", err);
//            telemetry.update();
//        }
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
              driveTrain[i][j].setPower(0);

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)

            {
              driveTrain[i][j].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              driveTrain[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }


    }
}
