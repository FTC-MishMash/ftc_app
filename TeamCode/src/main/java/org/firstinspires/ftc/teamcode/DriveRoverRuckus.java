package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;


@TeleOp(name = "Drive Rover Ruckus", group = "Iterative Opmode")
//@Disabled
public class DriveRoverRuckus extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    Robot robot;
    /*

     * Code to run ONCE when the driver hits INIT
     */

    @Override

    public void init() {


        telemetry.addData("Status", "Initialized");

        robot = new Robot(hardwareMap);

//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    double speed = 1;
    int a = 0;

    boolean shaft = false;
    boolean linear = false;
    int shaftEncoder = 0;
    int linearEncoder = 0;

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }


    @Override
    public void loop() {
        tankDriveTrainSetPower(speed);//מערכת הנעה רובוט
        telemetry.addData("motor [0][0]:  ", robot.driveTrain[0][0].getCurrentPosition());
        telemetry.addData("motor [0][1]:  ", robot.driveTrain[0][1].getCurrentPosition());
        telemetry.addData("motor [1][0]:  ", robot.driveTrain[1][0].getCurrentPosition());
        telemetry.addData("motor [1][1]:  ", robot.driveTrain[1][1].getCurrentPosition());
        telemetry.addData("shaft[0]:  ", robot.shaft[0].getCurrentPosition());
        telemetry.addData("shaft[1]:  ", robot.shaft[1].getCurrentPosition());
        telemetry.addData("imu", robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();


        //   else
        if (gamepad2.right_stick_y > 0) {
            linear = true;
            robot.linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.linear.setPower(1);
            linearEncoder = robot.linear.getCurrentPosition();
        } else if (gamepad2.right_stick_y < 0) {
            linear = true;
            robot.linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.linear.setPower(-1);
            linearEncoder = robot.linear.getCurrentPosition();

        } else if (gamepad2.right_stick_y == 0) {
            linear = false;


        } else if (linear == false) {

            robot.linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linear.setTargetPosition(linearEncoder);
            robot.linear.setPower(0.7);

        } else if (gamepad2.a) {

            robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[0].setTargetPosition(2400);
            robot.shaft[1].setTargetPosition(2400);
            robot.shaft[0].setPower(1);
            robot.shaft[1].setPower(1);
            if (robot.shaft[0].getCurrentPosition() >= 2380 && robot.shaft[1].getCurrentPosition() >= 2380) {
                robot.shaft[0].setPower(0);
                robot.shaft[1].setPower(0);
            } else if (gamepad2.y) {
                robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.shaft[0].setTargetPosition(200);
                robot.shaft[1].setTargetPosition(200);
                robot.shaft[0].setPower(1);
                robot.shaft[1].setPower(1);
                if (robot.shaft[0].getCurrentPosition() >= 2280 && robot.shaft[0].getCurrentPosition() >= 2280) {

                }


            }

        } else if (gamepad2.b) {

            robot.inTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearEncoder = -1000;

            robot.shaft[0].setTargetPosition(200);
            robot.shaft[1].setTargetPosition(200);
            robot.shaft[0].setPower(1);
            robot.shaft[1].setPower(1);
            robot.inTake.setPower(1);
            if (robot.shaft[0].getCurrentPosition() <= 200 && robot.shaft[1].getCurrentPosition() <= 200) {
                robot.linear.setPower(1);
            }
//
        } else if (gamepad1.a) {
            speed = 0.5;
        } else if (gamepad1.b) {
            speed = 1;
        } else if (shaft = true) {

            shaftEncoder = robot.shaft[0].getCurrentPosition();

        } else if (shaft = false) {
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[0].setTargetPosition(shaftEncoder);
            robot.shaft[1].setTargetPosition(shaftEncoder);
            robot.shaft[0].setPower(0.5);
            robot.shaft[1].setPower(0.5);


        } else if (gamepad2.left_stick_y < 0) {
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shaft = true;
            robot.shaft[0].setPower(1);
            robot.shaft[1].setPower(1);

        } else if (gamepad2.left_stick_y > 0) {
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shaft = true;
            robot.shaft[0].setPower(-1);
            robot.shaft[1].setPower(-1);

        } else if (gamepad2.left_stick_y == 0) {
            shaft = false;
        } else if (gamepad2.right_bumper) {
            robot.inTake.setPower(1);
        } else if (gamepad2.right_trigger != 0) {
            robot.inTake.setPower(-1);

        } else if (gamepad2.left_bumper) {

            shaftEncoder = 0;

            linearEncoder=6700;


        } else if (gamepad2.x) {
            shaftEncoder=0;

        } else if (gamepad2.left_trigger != 0) {
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[0].setTargetPosition(2550);
            robot.shaft[1].setTargetPosition(2550);
            robot.shaft[0].setPower(1);
            robot.shaft[1].setPower(1);
        } else {
            if (robot.shaft[0].getCurrentPosition() > 20 &&
                    robot.shaft[1].getCurrentPosition() > 20 &&
                    gamepad2.left_stick_y == 0) {
                robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            }
            robot.inTake.setPower(0);
            if (!robot.linear.isBusy())
                robot.linear.setPower(0);
            robot.shaft[0].setPower(0);
            robot.shaft[1].setPower(0);
        }


    }


    @Override
    public void stop() {

    }

    private Orientation getAngularOriention() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private void tankDriveTrainSetPower(double speed) {
        robot.driveTrain[0][1].setPower(speed * (-gamepad1.right_stick_y));
        robot.driveTrain[0][0].setPower(speed * (-gamepad1.left_stick_y));
        robot.driveTrain[1][1].setPower(speed * (-gamepad1.right_stick_y));
        robot.driveTrain[1][0].setPower(speed * (-gamepad1.left_stick_y));
    }
}