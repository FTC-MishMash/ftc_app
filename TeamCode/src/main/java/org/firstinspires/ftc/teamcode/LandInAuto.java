package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
@Disabled
public class LandInAuto extends LinearOpMode {
    Robot robot;

    public void LandInAuto() {
        telemetry.addData("pitch", getAngularOriention().thirdAngle);
        telemetry.update();

        /**
         * here the robot using shafts and gyroscope to be parallel with the ground
         */
        robot.linear.setTargetPosition(70);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setPower(0.7);
        sleep(400);
        robot.linear.setTargetPosition(0);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setPower(0.7);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[0].setPower(0.5);
        robot.shaft[1].setPower(0.5);
        while (opModeIsActive() && getAngularOriention().thirdAngle <= 0) {
            if (robot.linear.getCurrentPosition() <= 150)
                robot.linear.setPower(0);

            robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition() + 120);
            robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition() + 120);
            sleep(100);
            robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition() - 30);
            robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition() - 30);
            telemetry.addData("pitch", getAngularOriention().thirdAngle);
            telemetry.addData("linear encoder", robot.linear.getCurrentPosition());
            telemetry.update();
        }
        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);


        sleep(2000);

        /**
         * here the robot using the linear and encoders to det to the ground
         */
        robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition());
        robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition());

        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.shaft[0].setPower(1);
        robot.shaft[1].setPower(1);

        robot.linear.setTargetPosition(-1000);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setPower(0.6);
        sleep(300);

        robot.linear.setTargetPosition(robot.linear.getCurrentPosition() + 300);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setPower(1);
        sleep(250);

        robot.linear.setTargetPosition(-1000);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setPower(0.6);
        while (opModeIsActive() && robot.linear.getCurrentPosition() >= -950) {
            telemetry.addData("pitch", getAngularOriention().thirdAngle);
            telemetry.addData("linear encoder", robot.linear.getCurrentPosition());
            telemetry.update();
        }


        robot.linear.setPower(0);
        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);

        /**
         * from here on the robot is on the ground
         * and getting of the leander
         */

        setMotorPower(new double[][]{{0.3, 0.3}, {0.3, 0.3}});

        sleep(300);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        robot.linear.setTargetPosition(0);
        robot.shaft[0].setTargetPosition(50);
        robot.shaft[1].setTargetPosition(50);
        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear.setPower(1);
        sleep(700);
        /**
         * here the robot fold his arms to its original shape
         */
        robot.shaft[0].setPower(0.4);
        robot.shaft[1].setPower(0.4);
        double t0 = getRuntime();
        while (opModeIsActive() &&
                robot.shaft[0].isBusy() &&
                robot.shaft[1].isBusy() &&
                getRuntime() - t0 < 0.6) ;

        robot.linear.setPower(0);
        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);
        robot.linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotorPower(double[][] power) { //Stores the four drivetrain motors power in array
        for (int row = 0; opModeIsActive() && row < 2; row++)
            for (int col = 0; opModeIsActive() && col < 2; col++)
                robot.driveTrain[row][col].setPower(power[row][col]);
    }

    @Override

    /**
     * here the robot lock the motors to hanging
     */
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        motorLock();
        waitForStart();
        LandInAuto();
    }

    private Orientation getAngularOriention() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private void motorLock() {
        robot.linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shaft[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.linear.setTargetPosition(0);
        robot.shaft[0].setTargetPosition(0);
        robot.shaft[1].setTargetPosition(0);

        robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.linear.setPower(0.7);
        robot.shaft[0].setPower(1);
        robot.shaft[1].setPower(1);
        while (!isStarted()) {
            telemetry.addData("pitch", getAngularOriention().thirdAngle);
            telemetry.update();
        }
    }
}