package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Thread.sleep;


@TeleOp(name = "Drive Check", group = "Iterative Opmode")
//@Disabled
public class DriveRoverRuckusCheck extends OpMode {

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

        robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        tankDriveTrainSetPower();//מערכת הנעה רובוט
        telemetry.addData("motor [0][0]:  ", robot.driveTrain[0][0].getCurrentPosition());
        telemetry.addData("motor [0][1]:  ", robot.driveTrain[0][1].getCurrentPosition());
        telemetry.addData("motor [1][0]:  ", robot.driveTrain[1][0].getCurrentPosition());
        telemetry.addData("motor [1][1]:  ", robot.driveTrain[1][1].getCurrentPosition());
        telemetry.addData("shaft[0]:  ", robot.shaft[0].getCurrentPosition());
        telemetry.addData("shaft[1]:  ", robot.shaft[1].getCurrentPosition());
        telemetry.addData("imu", robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        telemetry.update();


//        if (gamepad2.a) {
//            motorLock();
//        }
//        //   else
         if (gamepad2.right_stick_y > 0) {
            robot.linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.linear.setPower(1);
        } else if (gamepad2.right_stick_y < 0) {
            robot.linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.linear.setPower(-1);

        }
//        else if (gamepad2.a){
//            robot.linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.linear.setTargetPosition(10);
//            robot.linear.setPower(1);
//            robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }else if(gamepad2.b){
//            robot.linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.linear.setTargetPosition(950);
//            robot.linear.setPower(1);
//            robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
        else if (gamepad2.left_stick_y < 0) {
//            robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.shaft[0].setPower(0.8);
            robot.shaft[1].setPower(0.8);
        } else if (gamepad2.left_stick_y > 0) {
//            robot.shaft[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.shaft[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.shaft[0].setPower(-0.8);
            robot.shaft[1].setPower(-0.8);

        } else if (gamepad2.right_bumper) {
            robot.inTake.setPower(1);
        } else if (gamepad2.right_trigger != 0) {
            robot.inTake.setPower(-1);
        } else if (gamepad2.left_bumper) {
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.shaft[0].setTargetPosition(0);
            robot.shaft[1].setTargetPosition(0);
            robot.shaft[0].setPower(1);
            robot.shaft[1].setPower(1);
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.linear.setTargetPosition(6700);
//            robot.linear.setPower(1);
//            robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad2.x) {
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.shaft[0].setTargetPosition(0);
            robot.shaft[1].setTargetPosition(0);
            robot.shaft[0].setPower(1);
            robot.shaft[1].setPower(1);
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }  else if (gamepad2.left_trigger !=0) {
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[0].setTargetPosition(2675);
            robot.shaft[1].setTargetPosition(2675);
            robot.shaft[0].setPower(1);
            robot.shaft[1].setPower(1);
        }
//        } else if (gamepad2.y) {
//            robot.shaft[0].setTargetPosition(7666);
//            robot.shaft[1].setTargetPosition(7666);
//            robot.shaft[0].setPower(1);
//            robot.shaft[1].setPower(1);
//            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        else {
            if ((robot.shaft[0].getCurrentPosition() > 20 &&
                    robot.shaft[1].getCurrentPosition() > 20 &&
                    gamepad2.left_stick_y == 0) ||
                    gamepad2.a) {
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


    void Sleep(int time) {
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void stop() {

    }

    public void motorLock() {
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
        while (!gamepad2.b) ;
    }

    private void tankDriveTrainSetPower() {
        robot.driveTrain[0][1].setPower(-gamepad1.right_stick_y);
        robot.driveTrain[0][0].setPower(-gamepad1.left_stick_y);
        robot.driveTrain[1][1].setPower(-gamepad1.right_stick_y);
        robot.driveTrain[1][0].setPower(-gamepad1.left_stick_y);
    }
}