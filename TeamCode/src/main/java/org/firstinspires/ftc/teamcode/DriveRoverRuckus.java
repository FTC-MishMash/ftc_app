package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;


public class DriveRoverRuckus {


    @TeleOp(name = "DriveRoverRuckus", group = "Iterative Opmode")
//@Disabled
    public class DRIVEY extends OpMode {
        double posRelic;
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        DcMotor[][] drivetrainDC = new DcMotor[3][3];
        DcMotor[] intakeDC = new DcMotor[2];
        DcMotor relicDC;
        double[][] sticks = new double[4][4];

        ModernRoboticsI2cRangeSensor range;

        double gain = 1;

        int pos[][] = new int[2][2];
        boolean relicStop = false;
        DigitalChannel touchUp;    //limit switch
        DigitalChannel touchDown;
        int a = 1;
        boolean flagSideOpen = true;
        boolean flagDoorMovingDown = false;
        boolean flagDoorMovingUp = false;
        boolean flagSideWait = false;
        boolean finishDoor = false;
        double tSideWait = 0;

        Boolean up = false;
        Boolean down = false;
        int count = 0;
        double posServo = 0;
        int relicEncoder;
        int relicPos;
        /*

         * Code to run ONCE when the driver hits INIT
         */

        @Override

        public void init() {

            telemetry.addData("Status", "Initialized");

            drivetrainDC[0][1] = hardwareMap.get(DcMotor.class, "rightFront");
            drivetrainDC[1][1] = hardwareMap.get(DcMotor.class, "rightBack");
            drivetrainDC[0][0] = hardwareMap.get(DcMotor.class, "leftFront");
            drivetrainDC[1][0] = hardwareMap.get(DcMotor.class, "leftBack");

            for (int i = 0; i < 1; i++) {

                for (int j = 0; j < 1; j++) {


                    drivetrainDC[i][j].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drivetrainDC[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

            }

            drivetrainDC[0][0].setDirection(DcMotorSimple.Direction.REVERSE);
            drivetrainDC[0][1].setDirection(DcMotorSimple.Direction.FORWARD);
            drivetrainDC[1][0].setDirection(DcMotorSimple.Direction.REVERSE);
            drivetrainDC[1][1].setDirection(DcMotorSimple.Direction.FORWARD);

            drivetrainDC[0][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drivetrainDC[0][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drivetrainDC[1][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drivetrainDC[1][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            relicDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drivetrainDC[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            relicDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            relicDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            intakeDC[1].setDirection(DcMotorSimple.Direction.REVERSE);
            telemetry.addData("Status", "Initialized");
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

            sticks[0][0] = gamepad1.left_stick_x;
            sticks[1][0] = gamepad1.left_stick_y;
            sticks[0][1] = gamepad1.right_stick_x;
            sticks[1][1] = gamepad1.right_stick_y;
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++) {
                    pos[i][j] = drivetrainDC[i][j].getCurrentPosition();

                }
            telemetry.update();
            double serGain = 0;
            Range.clip(1, 0, serGain);

            double strafeGain = 1;

                if (gamepad2.x || gamepad2.y)
                {


                    // WAIT FOR SIDE HOLDERS TO CLOSE
                    if (flagSideWait) {

                        if (gamepad2.x && getRuntime() - tSideWait > 0.4)
                        {
                            flagSideWait = false;
                            flagSideOpen = false;
                        } else if (gamepad2.y) {
                            flagSideWait = false;
                            flagSideOpen = false;
                        }
                    }


                if (gamepad2.right_trigger != 0) {
                    if (relicEncoder < 1500) {
                        relicDC.setTargetPosition(relicEncoder + 500);
                        relicDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        relicDC.setPower(1);
                        gain = 0.6;
                    } else if (relicEncoder < 2750) {
                        relicDC.setTargetPosition(relicEncoder + 200);
                        relicDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        relicDC.setPower(1);
                        gain = 0.6;
                    }

                }
                if (gamepad2.left_stick_button) {//auto relic
                    if (relicEncoder < 1750) {//open relic dc
                        relicDC.setTargetPosition(relicEncoder + 450);
                        relicDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        relicDC.setPower(1);
                    } else if (relicEncoder < 2325) {
                        relicDC.setTargetPosition(relicEncoder + 75);
                        relicDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        relicDC.setPower(1);
                    }
                }


            }
        }


        void tankDriveTrainSetPower(double gain) {
            drivetrainDC[0][0].setPower(gain * (gamepad1.left_stick_y));
            drivetrainDC[1][0].setPower(gain * (gamepad1.left_stick_y));
            drivetrainDC[0][1].setPower(gain * (gamepad1.right_stick_y));
            drivetrainDC[1][1].setPower(gain * (gamepad1.right_stick_y));
        }

        void strafe(double gain) {
            double trigger = 0;
            if (gain > 0)
                trigger = gamepad1.right_trigger;
            else if (gain < 0)
                trigger = gamepad1.left_trigger;

            drivetrainDC[0][1].setPower(gain * trigger);
            drivetrainDC[1][1].setPower(-gain * trigger);
            drivetrainDC[0][0].setPower(-gain * trigger);
            drivetrainDC[1][0].setPower(gain * trigger);
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
