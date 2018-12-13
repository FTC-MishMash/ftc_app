package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Thread.sleep;


@TeleOp(name = "Drive Rover Ruckus", group = "Iterative Opmode")
//@Disabled
public class DriveRoverRuckus extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor[] intakeDC = new DcMotor[2];
    double[][] sticks = new double[4][4];


    int pos[][] = new int[2][2];

    int a = 1;
    boolean flagSideOpen = true;

    boolean flagSideWait = false;

    double tSideWait = 0;

    Robot robot;
    /*

     * Code to run ONCE when the driver hits INIT
     */

    @Override

    public void init() {

        telemetry.addData("Status", "Initialized");

        robot = new Robot(hardwareMap);


//        robot.linear[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.linear[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain[0][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain[0][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain[1][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain[1][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
        tankDriveTrainSetPower(-1);

//            sticks[0][0] = gamepad1.left_stick_x;
//            sticks[1][0] = gamepad1.left_stick_y;
//            sticks[0][1] = gamepad1.right_stick_x;
//            sticks[1][1] = gamepad1.right_stick_y;
        telemetry.addData("imu", robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("[0][0]", robot.driveTrain[0][0].getCurrentPosition());
        telemetry.addData("[0][1]", robot.driveTrain[0][1].getCurrentPosition());
        telemetry.addData("[1][0]", robot.driveTrain[1][0].getCurrentPosition());
        telemetry.addData("[1][1]", robot.driveTrain[1][1].getCurrentPosition());
        telemetry.update();

//        if (gamepad2.a) {
//            robot.linear[0].setTargetPosition(6660);
//            robot.linear[1].setTargetPosition(6660);
//            robot.linear[0].setPower(0.75);
//            robot.linear[1].setPower(0.75);
//            robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            //linear goes down to the crater. הלינאריקה יורדת למכתש
//        }
//
//
//        if (gamepad2.b) {
//            robot.linear[0].setTargetPosition(0);
//            robot.linear[1].setTargetPosition(0);
//            robot.linear[0].setPower(0.75);
//            robot.linear[1].setPower(0.75);
//            robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            //linear goes  to the beginning. הלינאריקה חוזרת למצב התחלה
//        }
//
//        if (gamepad2.right_bumper) {
//            robot.inTake.setPower(1);
//            //intake turn on front. הציר של ההזנה פעול קדימה
//        } else if (gamepad2.left_bumper) {
//            robot.inTake.setPower(-1);
            //intake turn on back. הציר של ההזנה פועל אחורה
//        } else {
//            robot.inTake.setPower(0);
//        }


//        if (gamepad2.y) {
//            robot.linear[0].setTargetPosition(1000);
//            robot.linear[1].setTargetPosition(1000);
//            robot.linear[0].setPower(0.8);
//            robot.linear[1].setPower(0.8);
//            robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.shaft[0].setTargetPosition(4000);
//            robot.shaft[1].setTargetPosition(4000);
//            robot.shaft[0].setPower(0.8);
//            robot.shaft[1].setPower(0.8);
//            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            // הלינאריקה נסגרת עד ל1000 ואז הציר עולה לחצי (4000)
//        }
//
//        if (gamepad2.x) {
//            robot.shaft[0].setTargetPosition(0);
//            robot.shaft[1].setTargetPosition(0);
//            robot.shaft[0].setPower(0.75);
//            robot.shaft[1].setPower(0.75);
//            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            // shaft goes down to tha beginning. הציר יורד למצב התחלה
//        }

//        if (gamepad2.dpad_up) {
//            robot.shaft[0].setPower(0.25);
//            robot.shaft[1].setPower(0.25);
//            // ציר בלחיצה חופשית למעלה
//
//        } else if (gamepad2.dpad_down) {
//            robot.shaft[0].setPower(-0.25);
//            robot.shaft[1].setPower(-0.25);
//            //ציר בלחיצה חופשית למטה

//        } else {
//            robot.shaft[0].setPower(0);
//            robot.shaft[1].setPower(0);
//        }

        if (gamepad2.left_trigger < 0) {
            robot.shaft[0].setTargetPosition(8000);
            robot.shaft[1].setTargetPosition(8000);
            robot.shaft[0].setPower(0.8);
            robot.shaft[1].setPower(0.8);
            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // הציר
            robot.linear[0].setTargetPosition(6700);
            robot.linear[1].setTargetPosition(6700);
            robot.linear[0].setPower(0.75);
            robot.linear[1].setPower(0.75);
            robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //הלינאריקה
            // put the mineral inside the cargo. להזין את המינרלים לתןך הקארגו
        }
//        if (gamepad2.right_trigger < 0) {
//            robot.shaft[0].setTargetPosition(8000);
//            robot.shaft[1].setTargetPosition(8000);
//            robot.shaft[0].setPower(0.8);
//            robot.shaft[1].setPower(0.8);
//            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            //הציר
//            robot.linear[0].setTargetPosition(6700);
//            robot.linear[1].setTargetPosition(6700);
//            robot.linear[0].setPower(0.75);
//            robot.linear[1].setPower(0.75);
//            robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            //הלינאריקה
//            robot.shaft[0].setTargetPosition(16000);
//            robot.shaft[1].setTargetPosition(16000);
//            robot.shaft[0].setPower(0.8);
//            robot.shaft[1].setPower(0.8);
//            robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            //הציר
//            //טיפוס!!
//       }
//        if (gamepad2.dpad_right) {
//            robot.linear[0].setTargetPosition(6660);
//            robot.linear[1].setTargetPosition(6660);
//            robot.linear[0].setPower(0.75);
//            robot.linear[1].setPower(0.75);
//            robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.inTake.setPower(1);
//            // לינאריקה והזנה
//        }


    }
    void tankDriveTrainSetPower(double power) {
        robot.driveTrain[0][0].setPower(-power * (gamepad1.left_stick_y));
        robot.driveTrain[1][0].setPower(-power * (gamepad1.left_stick_y));
        robot.driveTrain[0][1].setPower(-power * (gamepad1.right_stick_y));
        robot.driveTrain[1][1].setPower(-power * (gamepad1.right_stick_y));
    }


    void glyphsServoOperation(int index, double position) {
//            servosGlyph[index].setPosition(position);
    }

    void intakeOperation(double power) {
        int i = 0;
        for (i = 0; i <= 1; i++)
            intakeDC[i].setPower(power);
    }

    void intakeNew(double power1, double power2) {


        intakeDC[0].setPower(power1);
        intakeDC[1].setPower(power2);
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


}