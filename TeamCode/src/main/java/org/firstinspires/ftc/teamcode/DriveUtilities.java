package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveUtilities {
    AutoMode currOpmode;
    Robot robot;
    BNO055IMU imu;
    DcMotor[][] motors;
    Telemetry telemetry;
    ImageTargets targetsNav;

    public DriveUtilities(AutoMode currOpmode) {
        this.currOpmode = currOpmode;
        this.robot = currOpmode.robot;
        this.imu = robot.imu;
        this.motors = robot.driveTrain;
        this.telemetry = currOpmode.telemetry;
    }

    public static void setMotorPower(DcMotor[][] driveTrain, double[][] power) { //Stores the four drivetrain motors power in array
        for (int row = 0; row < driveTrain.length; row++)
            for (int col = 0; col < driveTrain[row].length; col++)
                driveTrain[row][col].setPower(power[row][col]);
    }

    public void scaledTurn(double goalAngle, double power) {
        boolean sideOfTurn = true;
        double deltaAngle = 0;

        boolean directTurn = true;
        double currentAngle = normalizedAngle(getAngularOriention(imu).firstAngle);
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
            setMotorPower(motors, new double[][]{{power, -power}, {power, -power}});
        else
            setMotorPower(motors, new double[][]{{-power, power}, {-power, power}});
        if (directTurn)
            while (currOpmode.opModeIsActive() && Math.abs(angle0 - currentAngle) < deltaAngle) {  //motors running
                currentAngle = normalizedAngle(getAngularOriention(imu).firstAngle);
                telemetry.addData("angle case 3:", currentAngle);
                telemetry.update();
            }
        else if (goalAngle > 180 && currentAngle < 180)
            while (currOpmode.opModeIsActive() &&
                    (currentAngle <= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle > 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                currentAngle = normalizedAngle(getAngularOriention(imu).firstAngle);
                telemetry.addData("angle case 1:", currentAngle);
                telemetry.update();
            }

        else if (goalAngle < 180 && currentAngle > 180)
            while (currOpmode.opModeIsActive() && (currentAngle >= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle < 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                currentAngle = normalizedAngle(getAngularOriention(imu).firstAngle);
                telemetry.addData("angle case 2:", currentAngle);
                telemetry.update();
            }


        setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});
    }

    public void driveByEncoderRoverRuckus(int goalDistRight, int goalDistLeft, double power, boolean targets) {// Drive by encoders and converts incoders ticks to distance in cm and drives until distance is completed.
//direction 0 is forword, 1 is backword
        final int tixRound = 600;
        final int cmRound = 27;
        this.targetsNav=currOpmode.targetNav;


        int dRight = (goalDistRight * tixRound) / cmRound;
        int dLeft = (goalDistLeft * tixRound) / cmRound;

        robot.driveTrain[0][0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain[1][0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.driveTrain[0][1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.driveTrain[1][1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.driveTrain[0][0].setTargetPosition(robot.driveTrain[0][0].getCurrentPosition() + dLeft);
        robot.driveTrain[1][0].setTargetPosition(robot.driveTrain[1][0].getCurrentPosition() + dLeft);
//        robot.driveTrain[0][1].setTargetPosition(robot.driveTrain[0][1].getCurrentPosition() + dRight);
//        robot.driveTrain[1][1].setTargetPosition(robot.driveTrain[1][1].getCurrentPosition() + dRight);


        robot.driveTrain[0][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveTrain[1][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain[0][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.driveTrain[1][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                robot.driveTrain[i][j].setPower(power);

        telemetry.addLine("go to target");
        telemetry.update();

        double runTime = currOpmode.getRuntime();
        while (currOpmode.opModeIsActive() &&
                robot.driveTrain[0][0].isBusy()
                && robot.driveTrain[1][0].isBusy()
//                && robot.driveTrain[0][1].isBusy()
//                && robot.driveTrain[1][1].isBusy()
                && currOpmode.getRuntime() - runTime < Math.abs((dRight + dLeft / 2) / 10)) {
            if(targets&&targetsNav.getPositions()!=null)
                break;
            currOpmode.sleep(0);
        }


        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                robot.driveTrain[i][j].setPower(0);

        telemetry.addLine("end move encoder");
        telemetry.update();


        robot.driveTrain[0][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain[1][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain[0][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain[1][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public static double normalizedAngle(double angle) {
        if (angle < 0) {
            while (angle < 0)
                angle += 360;
        } else if (angle >= 360) {
            while (angle >= 360)
                angle -= 360;
        }
        return angle;
    }

    public void diffTurn(double diffAngle, double power) {
        double currAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double goalAngle = normalizedAngle(diffAngle + currAngle);
        scaledTurn(goalAngle, 0.4);

    }

    public static Orientation getAngularOriention(BNO055IMU imu) {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
