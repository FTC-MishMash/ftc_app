package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Disabled
public class checkThing extends AutoMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    int angle = 0;
    int targetEncoder = 0;
    double power = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void runOpMode() throws InterruptedException {
//
        telemetry.addData("Status", "Initialized");
        //super.runOpMode();
        super.runOpMode();


        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                angle += 5;
                sleep(700);
            } else if (gamepad1.dpad_down) {
                angle -= 5;
                sleep(700);
            } else if (gamepad1.dpad_right) {
                targetEncoder += 5;
                sleep(700);
            } else if (gamepad1.dpad_left) {
                targetEncoder -= 5;
                sleep(700);
            } else if (gamepad1.right_bumper) {
                power += 0.05;
                sleep(700);
            } else if (gamepad1.left_bumper) {
                power -= 0.05;
                sleep(700);
            }
            if (gamepad1.x) {
                driveUtils.TurnWithEncoder(angle, power);
            } else if (gamepad1.y) {
                driveUtils.driveByEncoderRoverRuckus(targetEncoder, targetEncoder, power, false);
            }
            telemetry.addData("angle", angle);
            telemetry.addData("power", power);
            telemetry.addData("target encoder", targetEncoder);
            telemetry.update();
        }

    }
}