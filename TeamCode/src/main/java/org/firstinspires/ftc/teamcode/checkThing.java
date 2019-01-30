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


@TeleOp(name = "check thing", group = "Iterative Opmode")
//@Disabled
public class checkThing extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DriveUtilities driveUtils;
    Robot robot;
    int angle = 0;
    int targetEncoder = 0;
    double power = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override

    public void init() {

//
        telemetry.addData("Status", "Initialized");

        robot = new Robot(hardwareMap);

    }


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
        if (gamepad1.dpad_up) {
            angle += 5;
            Sleep(700);
        } else if (gamepad1.dpad_down) {
            angle -= 5;
            Sleep(700);
        } else if (gamepad1.dpad_right) {
            targetEncoder += 5;
            Sleep(700);
        } else if (gamepad1.dpad_left) {
            angle -= 5;
            Sleep(700);
        } else if (gamepad1.right_bumper) {
            power += 0.05;
            Sleep(700);
        } else if (gamepad1.left_bumper) {
            power -= 0.05;
            Sleep(700);
        }
        if (gamepad1.x) {
            driveUtils.TurnWithEncoder(angle, power, 0.3);
        } else if (gamepad1.y) {
            driveUtils.driveByEncoderRoverRuckus(targetEncoder, targetEncoder, power, false);
        }
        telemetry.addData("angle",angle);
        telemetry.addData("power",power);
        telemetry.addData("target encoder",targetEncoder);
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

    void Sleep(int time) {
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}