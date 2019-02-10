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
//@Disabled
public class checkThing extends AutoMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    int angle = 0;
    int targetEncoder = 0;
    double power = 0;
    AutoMode auto;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override

    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        robot = new Robot(hardwareMap);
        targetNav = new ImageTargets(this);
        driveUtils = new DriveUtilities(this);
        tsSampling = new TensorflowUtils(this);


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("left front motor CM", robot.driveTrain[0][0].getCurrentPosition() * 27 / 600);
            telemetry.addData("left back motor CM", robot.driveTrain[1][0].getCurrentPosition() * 27 / 600);
            telemetry.addData("right fornt motor CM", robot.driveTrain[0][1].getCurrentPosition() * 27 / 600);
            telemetry.addData("right back motor CM", robot.driveTrain[1][1].getCurrentPosition() * 27 / 600);


            telemetry.addData("left front motor encoder", robot.driveTrain[0][0].getCurrentPosition());
            telemetry.addData("left back motor encoder", robot.driveTrain[1][0].getCurrentPosition());
            telemetry.addData("right fornt motor encoder", robot.driveTrain[0][1].getCurrentPosition());
            telemetry.addData("right back motor encoder", robot.driveTrain[1][1].getCurrentPosition());
            telemetry.addData("angle: ", angle);
            telemetry.addData("power: ", power);
            telemetry.addData("imu angle", DriveUtilities.normalizedAngle(DriveUtilities.getAngularOriention(robot.imu).firstAngle));
////
//            telemetry.addData("shaft right CM", robot.shaft[0].getCurrentPosition()*27/600);
//            telemetry.addData("shaft left CM", robot.shaft[1].getCurrentPosition()*27/600);
//            telemetry.addData("shaft right encoder", robot.shaft[0].getCurrentPosition());
//            telemetry.addData("shaft left encoder", robot.shaft[1].getCurrentPosition());
//            telemetry.addData("linear CM", robot.linear.getCurrentPosition()*27/600);
//            telemetry.addData("linear encoder", robot.linear.getCurrentPosition());
            telemetry.update();

            if (gamepad1.dpad_up) {
                angle += 5;
                sleep(200);
            } else if (gamepad1.dpad_down) {
                angle -= 5;
                sleep(200);
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

            }
           else if (gamepad1.right_stick_button)
                driveUtils.scaledTurn(angle, power);
            else if (gamepad1.y) {
                driveUtils.driveByEncoderRoverRuckus(targetEncoder, targetEncoder, power, false);
            }
            if (gamepad1.right_trigger > 0) {

                driveUtils.driveByEncoderRoverRuckus(robot.distToDepot, robot.distToDepot, -robot.powerEncoder, false);//to depot
                Marker(0.5, robot.shaftTargetPositionMarker);  //marker
                // driveUtils.driveByEncoderRoverRuckus(90, 90, -0.5, false);//to crater
                driveUtils.driveByEncoderRoverRuckus(robot.distToCrater, robot.distToCrater, robot.powerEncoder, false);//to crater

            } else if (gamepad1.left_trigger > 0) {
                int cubePosition = 1;
                targetNav.startTracking();
//            sleep(500);

                float[] pos = targetNav.getPositions();
                telemetry.addData("pos null: ", pos == null);
                telemetry.update();
//            sleep(500);
                if (pos == null) {

                    telemetry.addData("start searching wait for click", cubePosition);
                    telemetry.update();
//                sleep(500);
                    targetNav.searchImage(cubePosition, -0.20);
                } else if (gamepad1.b) {
                    driveUtils.back_up_driveByImage(0.45, robot.AngleToDepot, 95);
                }


//            sleep(600);
                targetNav.driveToImage(-0.3);
//

            }

        }
    }
}