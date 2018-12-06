package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;


public class DriveRoverRuckus {


    @TeleOp(name = "DriveRoverRuckus", group = "Iterative Opmode")
//@Disabled
    public class DRIVEY extends OpMode {

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


            telemetry.addData("Status", "Initialized");

            robot.linear[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.linear[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            telemetry.update();

        }


        @Override
        public void loop() {
            tankDriveTrainSetPower(1);
//            sticks[0][0] = gamepad1.left_stick_x;
//            sticks[1][0] = gamepad1.left_stick_y;
//            sticks[0][1] = gamepad1.right_stick_x;
//            sticks[1][1] = gamepad1.right_stick_y;

            telemetry.update();
            double serGain = 0;
            Range.clip(1, 0, serGain);


            if (gamepad2.a) {
                robot.linear[0].setTargetPosition(6660);
                robot.linear[1].setTargetPosition(6660);
                robot.linear[0].setPower(0.75);
                robot.linear[1].setPower(0.75);
                robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.b) {
                robot.linear[0].setTargetPosition(0);
                robot.linear[1].setTargetPosition(0);
                robot.linear[0].setPower(0.75);
                robot.linear[1].setPower(0.75);
                robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.right_bumper) {
                robot.inTake.setPower(1);
            } else robot.inTake.setPower(0);

            if (gamepad2.y) {
                robot.shaft[0].setTargetPosition(7666);
                robot.shaft[1].setTargetPosition(7666);
                robot.shaft[0].setPower(0.8);
                robot.shaft[1].setPower(0.8);
                robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.x) {
                robot.shaft[0].setTargetPosition(0);
                robot.shaft[1].setTargetPosition(0);
                robot.shaft[0].setPower(0.75);
                robot.shaft[1].setPower(0.75);
                robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (gamepad2.dpad_up) {
                robot.shaft[0].setPower(0.25);
                robot.shaft[1].setPower(0.25);
            }
            if (gamepad2.dpad_down) {
                robot.shaft[0].setPower(-0.25);
                robot.shaft[1].setPower(-0.25);
            } else {
                robot.shaft[0].setPower(0);
                robot.shaft[1].setPower(0);
            }
            if (gamepad2.left_bumper) {
                robot.shaft[0].setTargetPosition(8000);
                robot.shaft[1].setTargetPosition(8000);
                robot.shaft[0].setPower(0.8);
                robot.shaft[1].setPower(0.8);
                robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.linear[0].setTargetPosition(6700);
                robot.linear[1].setTargetPosition(6700);
                robot.linear[0].setPower(0.75);
                robot.linear[1].setPower(0.75);
                robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

        }


        void tankDriveTrainSetPower(double gain) {
            robot.driveTrain[0][0].setPower(gain * (gamepad1.left_stick_y));
            robot.driveTrain[1][0].setPower(gain * (gamepad1.left_stick_y));
            robot.driveTrain[0][1].setPower(gain * (gamepad1.right_stick_y));
            robot.driveTrain[1][1].setPower(gain * (gamepad1.right_stick_y));
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

}