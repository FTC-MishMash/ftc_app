package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This class is used for performing all driving functions.
 */
public class DriveUtilities {
    AutoMode currOpmode; //Current autonomous opmode running.
    Robot robot;
    BNO055IMU imu;
    DcMotor[][] motors;
    Telemetry telemetry;
    ImageTargets targetsNav;

    /**
     * Initializing the all the data members; robot and opmode components for the DriveUtilities instance.
     *
     * @param currOpmode-
     */
    public DriveUtilities(AutoMode currOpmode) {
        this.currOpmode = currOpmode;
        this.robot = currOpmode.robot;
        this.imu = currOpmode.robot.imu;
        this.motors = robot.driveTrain;
        this.telemetry = currOpmode.telemetry;
    }

    /**
     * Setting power to robot motors.
     *
     * @param driveTrain- robot driving motors.
     * @param power-      amount of required power for driving.
     */
    public static void setMotorPower(DcMotor[][] driveTrain, double[][] power) { //Stores the four drivetrain motors power in array
        for (int row = 0; row < driveTrain.length; row++)
            for (int col = 0; col < driveTrain[row].length; col++)
                driveTrain[row][col].setPower(power[row][col]);
    }

    /**
     * Rotating thr robot to desired angle using the imu.
     *
     * @param goalAngle- target angle for rotating.
     * @param power-     motors power while rotating, should be positive.
     */
    public void scaledTurn(double goalAngle, double power) {
        boolean sideOfTurn = true;
        double deltaAngle = 0;

        boolean directTurn = true;
        double currentAngle = normalizedAngle(getAngularOriention(imu).firstAngle);
        telemetry.addData("start angle: ", currentAngle);
        telemetry.update();
        currOpmode.sleep(500);
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
                //   power -= 0.001;
                currentAngle = normalizedAngle(getAngularOriention(imu).firstAngle);
                telemetry.addData("angle case 3:", currentAngle);
                telemetry.update();
            }
        else if (goalAngle > 180 && currentAngle < 180)
            while (currOpmode.opModeIsActive() &&
                    (currentAngle <= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle > 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                //     power -= 0.001;
                currentAngle = normalizedAngle(getAngularOriention(imu).firstAngle);
                telemetry.addData("angle case 1:", currentAngle);
                telemetry.update();
            }

        else if (goalAngle < 180 && currentAngle > 180)
            while (currOpmode.opModeIsActive() && (currentAngle >= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle < 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                //   power -= 0.001;
                currentAngle = normalizedAngle(getAngularOriention(imu).firstAngle);
                telemetry.addData("angle case 2:", currentAngle);
                telemetry.update();
            }
        telemetry.addData("angle completed: ", normalizedAngle(getAngularOriention(imu).firstAngle));
        telemetry.update();
        currOpmode.sleep(4500);
        setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});
        telemetry.addData("Start encoders:", normalizedAngle(getAngularOriention(imu).firstAngle));
        telemetry.update();
        currOpmode.sleep(4000);
    }

    /**
     * Given a distance for driving this method converts the encoders poition to distance until target distance is
     * completed.
     * @param goalDistRight target distance for the right side motors.
     * @param goalDistLeft target distance for the left side motors.
     * @param power power value and direction for the drivetrain motors.
     * @param targets should be true when the robot search image targets during the drive.
     */
    public void driveByEncoderRoverRuckus(int goalDistRight, int goalDistLeft, double power, boolean targets) {// Drive by encoders and converts incoders ticks to distance in cm and drives until distance is completed.

        final int tixRound = 600;
        final int cmRound = 27;
        this.targetsNav = currOpmode.targetNav;


        int dRight = (goalDistRight * tixRound) / cmRound;
        int dLeft = (goalDistLeft * tixRound) / cmRound;

        robot.driveTrain[0][0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain[1][0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain[0][1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain[1][1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.driveTrain[0][0].setTargetPosition(robot.driveTrain[0][0].getCurrentPosition() + dLeft);
        robot.driveTrain[1][0].setTargetPosition(robot.driveTrain[1][0].getCurrentPosition() + dLeft);

        robot.driveTrain[0][1].setTargetPosition(robot.driveTrain[0][1].getCurrentPosition() + dRight);
        robot.driveTrain[1][1].setTargetPosition(robot.driveTrain[1][1].getCurrentPosition() + dRight);


        robot.driveTrain[0][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveTrain[1][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveTrain[0][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveTrain[1][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                robot.driveTrain[i][j].setPower(power);

        telemetry.addLine("go to target");
        telemetry.update();

        double runTime = currOpmode.getRuntime();
        while (currOpmode.opModeIsActive() &&
                robot.driveTrain[0][0].isBusy()
                && robot.driveTrain[1][0].isBusy()
                && robot.driveTrain[0][1].isBusy()
                && robot.driveTrain[1][1].isBusy()
                && currOpmode.getRuntime() - runTime < Math.abs((dRight + dLeft / 2) / 10)) {

            if (targets && targetsNav.getPositions() != null) {
                setMotorPower(robot.driveTrain, new double[][]{{0, 0}, {0, 0}});
                telemetry.addData("pos drive null: ", currOpmode.targetNav.currOpmode == null);
                telemetry.update();
                currOpmode.sleep(2000);
                break;
            }
        }
        setMotorPower(robot.driveTrain, new double[][]{{0, 0}, {0, 0}});

//        for (int i = 0; i < 2; i++)
//            for (int j = 0; j < 2; j++)
//                robot.driveTrain[i][j].setPower(0);

        telemetry.addLine("end move encoder");
        telemetry.update();


        robot.driveTrain[0][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain[1][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain[0][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain[1][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.driveTrain[0][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain[1][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain[0][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void TurnWithEncoder(double goalAngle, double power) {
        DcMotor[][] driveMotors = robot.driveTrain;
        boolean sideOfTurn = true;
        double deltaAngle = 0;
        int motorsCurrentEncoder[][] = new int[2][2];
        boolean directTurn = true;
        double currentAngle = normalizedAngle(getAngularOriention(robot.imu).firstAngle);
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
            power *= -1;
        double decAngle=0.55;
        double decMotors=0.8;
        setMotorPower(robot.driveTrain, new double[][]{{power, -power}, {power, -power}});
        if (directTurn)
            while (currOpmode.opModeIsActive() && Math.abs(angle0 - currentAngle) < deltaAngle) {  //motors running
                currentAngle = normalizedAngle(getAngularOriention(robot.imu).firstAngle);
//                if(Math.abs(angle0 - currentAngle) <= deltaAngle*decAngle)
//                {
//                    decAngle+=0.08;
//                    power*=decMotors;
//                    decMotors*=decMotors;
//                }
                telemetry.addData("angle case 3:", currentAngle);
                telemetry.update();
            }
        else if (goalAngle > 180 && currentAngle < 180)
            while (currOpmode.opModeIsActive() &&
                    (currentAngle <= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle > 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                currentAngle = normalizedAngle(getAngularOriention(robot.imu).firstAngle);
//                if(Math.max(Math.abs(angle0 - currentAngle),360 - Math.abs((angle0 - currentAngle)))< deltaAngle*decAngle)
//                {
//                    decAngle+=0.08;
//                    power*=decMotors;
//                    decMotors*=decMotors;
//                }
                telemetry.addData("angle case 1:", currentAngle);
                telemetry.update();
            }

        else if (goalAngle < 180 && currentAngle > 180)
            while (currOpmode.opModeIsActive() && (currentAngle >= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle < 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                currentAngle = normalizedAngle(getAngularOriention(robot.imu).firstAngle);
//                if(Math.max(Math.abs(angle0 - currentAngle),360 - Math.abs((angle0 - currentAngle)))< deltaAngle*decAngle)
//                {
//                    decAngle+=0.08;
//                    power*=decMotors;
//                    decMotors*=decMotors;
//                }
                telemetry.addData("angle case 2:", currentAngle);
                telemetry.update();
            }
        setMotorPower(driveMotors, new double[][]{{0, 0}, {0, 0}});
        telemetry.addData("Start encoders:", normalizedAngle(getAngularOriention(imu).firstAngle));
        telemetry.update();
        currOpmode.sleep(4000);
//        for (int i = 0; i < 2; i++)
//            for (int j = 0; j < 2; j++)
//                motorsCurrentEncoder[i][j] = driveMotors[i][j].getCurrentPosition();
//        for (int i = 0; i < 2; i++)
//            for (int j = 0; j < 2; j++) {
//                driveMotors[i][j].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                driveMotors[i][j].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                driveMotors[i][j].setTargetPosition(motorsCurrentEncoder[i][j]);
//                setMotorPower(driveMotors, new double[][]{{power, power}, {power, power}});
//            }
//        while (currOpmode.opModeIsActive() &&
//                driveMotors[0][0].isBusy()
//                && driveMotors[1][0].isBusy()
//                && driveMotors[0][1].isBusy()
//                && driveMotors[1][1].isBusy()) {
//            telemetry.addData("Encoder busy:", currentAngle);
//            telemetry.update();
//        }
//        for (int i = 0; i < 2; i++)
//            for (int j = 0; j < 2; j++)
//                driveMotors[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorPower(driveMotors, new double[][]{{0, 0}, {0, 0}});
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

    public void back_up_driveByImage(double power, int turnAngle, int driveDist) {
        driveByEncoderRoverRuckus(driveDist, driveDist, 0.7, false);
        TurnWithEncoder(turnAngle, power);
    }

    public void diffTurn(double diffAngle, double power) {
        double currAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double goalAngle = normalizedAngle(diffAngle + currAngle);
        TurnWithEncoder(goalAngle, power);

    }

    public static Orientation getAngularOriention(BNO055IMU imu) {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void DriveByDistance(double distance, double power, int correct) {
        double pidErr[] = {0, 0}; //PID reset

        boolean range0 = true; //in case the ranaesensor is detacahed range0 is false
        Rev2mDistanceSensor rangeSensor = robot.rangeSensor;
        for (int ii = 0; ii < correct + 1; ii++) {
            if (ii > 0)
                power *= 0.75;
            if (distance > rangeSensor.getDistance(DistanceUnit.CM)) {// Further then tareget distance
                telemetry.addData("HERE", rangeSensor.getDistance((DistanceUnit.CM)));
                telemetry.update();
                double time = currOpmode.getRuntime();
                while (rangeSensor.getDistance(DistanceUnit.CM) != 0 && currOpmode.opModeIsActive() && (rangeSensor.getDistance(DistanceUnit.CM) + 2 < distance) && currOpmode.getRuntime() < (time + distance / 10)) {
                    //     pidErr = GyroPID(heading, pidErr[1]);
                    setMotorPower(motors, new double[][]{{-power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], -power + pidErr[0]}});
                    telemetry.addData("FromWall", rangeSensor.getDistance((DistanceUnit.CM)));
                    telemetry.update();
//                sleep(200);
                }
            } else if ((distance) < rangeSensor.getDistance(DistanceUnit.CM)) {//Closer then tareget distance
                double time = currOpmode.getRuntime();

                while (rangeSensor.getDistance(DistanceUnit.CM) != 0 && currOpmode.opModeIsActive() && (distance < rangeSensor.getDistance(DistanceUnit.CM) - 2) && currOpmode.getRuntime() < (time + distance / 10)) {
                    //  pidErr = GyroPID(heading, pidErr[1]);
                    setMotorPower(motors, new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
                    telemetry.addData("ToWall", rangeSensor.getDistance((DistanceUnit.CM)));
                    telemetry.update();
                }

            }
        }
        setMotorPower(motors, new double[][]{{0, 0}, {0, 0}});  //stop drivetrain motors
        if (rangeSensor.getDistance(DistanceUnit.CM) == 0) { //In case the range sensor is detached.
            range0 = false;

        }
    }
}
