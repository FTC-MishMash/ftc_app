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
    // Declare OpModem embers.
    private ElapsedTime runtime = new ElapsedTime();

    Robot robot;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override

    public void init() {


        telemetry.addData("Status", "Initialized");

        robot = new Robot(hardwareMap);
//        robot.linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    double speed = 1;


    boolean shaft = false;
    boolean linear = false;
    int shaftEncoder = 0;
    int linearEncoder = 0;
    double shaftPower = 0.5;
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
        tankDriveTrainSetPower(speed);

// SHAFT MANUAL MOVEMENT
        if (gamepad2.right_stick_y != 0) {//hand mode shaft turn ON
            shaft = true;
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.shaft[0].setPower((gamepad2.right_stick_y) * (0.5));
            robot.shaft[1].setPower((gamepad2.right_stick_y) * (0.5));

            shaftEncoder = robot.shaft[1].getCurrentPosition();
        } else
            shaft = false;

        if (gamepad2.x) {
            shaftEncoder = robot.shaftTargetPositionMarker;
            shaftPower = 1;
            robot.hanging.setPosition(robot.hangingLockPosition);
        }


        if (gamepad2.left_stick_y > 0) {//hand - linear

//            linear = true;
            robot.linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.linear.setPower(gamepad2.left_stick_y);
//            linearEncoder = robot.linear.getCurrentPosition();

        } else if (gamepad2.left_stick_y < 0) {
//            linear = true;

            robot.linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.linear.setPower(gamepad2.left_stick_y);
//            linearEncoder = robot.linear.getCurrentPosition();

        } else if (gamepad2.left_stick_y == 0) {//hand mode turn OFF
            linear = false;

        }
//        robot.linear.setPower((gamepad2.left_stick_y));

//        else if (gamepad2.right_stick_y < 0) {//hand - linear
//            linear = true;
//            robot.linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.linear.setPower(-1);
//            linearEncoder = robot.linear.getCurrentPosition();
//        }


        if (gamepad2.a) {//90 deg shaft. go to lander

            shaftEncoder = robot.shaftEncoderPosition90deg;
            linearEncoder = robot.linearClosePosition;
        }
        if (gamepad2.b) {//go to crater and intake minerals

            shaftEncoder = robot.shaftEncoderPositionINTAKE;

//            if (robot.shaft[0].getCurrentPosition() <= robot.shaftMiddlePosition) {
            linearEncoder = robot.linearOpenPosition;
            robot.inTake.setPower(1);
//            }

        }
// else if (gamepad2.left_stick_y != 0) {
//            shaft = true;
//            shaftEncoder = robot.shaft[0].getCurrentPosition();
//        } else if (gamepad2.right_stick_y != 0) {
//            linear = true;
//            linearEncoder = robot.linear.getCurrentPosition();


        if (gamepad2.right_bumper) {
            robot.inTake.setPower(-1);
        }
        if (gamepad2.right_trigger != 0) {
            robot.inTake.setPower(1);

        }
        if (gamepad2.left_trigger > 0) {

            robot.hanging.setPosition(robot.hangingOpenPosition);


        } else if (gamepad2.left_bumper) {
            robot.hanging.setPosition(robot.hangingLockPosition);

        }
        if (gamepad2.y) {//reset encoders
            DcMotor.RunMode currMode = robot.shaft[0].getMode();
            robot.shaft[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.shaft[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.shaft[0].setMode(currMode);
            robot.shaft[1].setMode(currMode);
            shaftEncoder = 0;
        }

        if (!gamepad2.right_bumper && gamepad2.right_trigger == 0 && !gamepad2.y)//TODO: add all the button that use intake
            robot.inTake.setPower(0);
//TODO: check if Aviad prefer button TURN OFF for the intake

        if (!shaft) {//in progress when AUTO mode shaft turn ON

            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[0].setTargetPosition(shaftEncoder);
            robot.shaft[1].setTargetPosition(shaftEncoder);

            robot.shaft[0].setPower(shaftPower);
            robot.shaft[1].setPower(shaftPower);
        }

        if (!linear) {//in progress when AUTO mode turn ON

            robot.linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linear.setTargetPosition(linearEncoder);
            robot.linear.setPower(0.7);

        }

        telemetry.addData("motor [0][0]:  ", robot.driveTrain[0][0].getCurrentPosition());
        telemetry.addData("motor [0][1]:  ", robot.driveTrain[0][1].getCurrentPosition());
        telemetry.addData("motor [1][0]:  ", robot.driveTrain[1][0].getCurrentPosition());
        telemetry.addData("motor [1][1]:  ", robot.driveTrain[1][1].getCurrentPosition());
        telemetry.addData("shaft[0]:  ", robot.shaft[0].getCurrentPosition());
        telemetry.addData("shaft[1]:  ", robot.shaft[1].getCurrentPosition());
        telemetry.addData("shaft flag: ", shaft);
//        telemetry.addData("linear flag: ",linear);
        telemetry.addData("imu", robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
    }


    @Override
    public void stop() {
        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);
        robot.linear.setPower(0);
        robot.inTake.setPower(0);
        robot.driveTrain[0][1].setPower(0);
        robot.driveTrain[0][0].setPower(0);
        robot.driveTrain[1][1].setPower(0);
        robot.driveTrain[1][0].setPower(0);
    }

    private Orientation getAngularOriention() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private void tankDriveTrainSetPower(double speed) {

        if (gamepad1.right_trigger > 0)
            speed = 0.7;
        else
            speed = 1;

        robot.driveTrain[0][1].setPower(speed * (-gamepad1.left_stick_y));
        robot.driveTrain[0][0].setPower(speed * (-gamepad1.right_stick_y));
        robot.driveTrain[1][1].setPower(speed * (-gamepad1.left_stick_y));
        robot.driveTrain[1][0].setPower(speed * (-gamepad1.right_stick_y));
    }
}