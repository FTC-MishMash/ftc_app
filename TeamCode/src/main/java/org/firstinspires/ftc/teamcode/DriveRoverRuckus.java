package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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

                }
                if (gamepad2.a ){
                    robot.linear[0].setTargetPosition(6660);
                    robot.linear[1].setTargetPosition(6660);
                    robot.linear[0].setPower(0.75);
                    robot.linear[1].setPower(0.75);
                    robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad2.b ){
                    robot.linear[0].setTargetPosition(0);
                    robot.linear[1].setTargetPosition(0);
                    robot.linear[0].setPower(0.75);
                    robot.linear[1].setPower(0.75);
                    robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linear[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
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


        /**
         * Created by user on 30/10/2018.
         */

        public static class Driving {
            public static void ScaledTurn(double goalAngle, DcMotor[][] driveMotors, BNO055IMU imu, double power, Telemetry telemetry) {
                boolean sideOfTurn = true;
                double deltaAngle = 0;
                boolean directTurn = true;
                double currentAngle = getCurrentScaledAngle(imu);
                double angle0 = currentAngle;
                if (currentAngle < goalAngle) {
                    if (goalAngle - currentAngle <= 360 - (goalAngle - currentAngle)) {
                        sideOfTurn = false;
                        deltaAngle = goalAngle - currentAngle;
                    } else {
                        sideOfTurn = true;
                        deltaAngle = 360 - (goalAngle - currentAngle);
                        directTurn = false;
                    }


                } else {
                    if (currentAngle - goalAngle <= 360 - (currentAngle - goalAngle)) {
                        sideOfTurn = true;
                        deltaAngle = currentAngle - goalAngle;
                    } else {
                        sideOfTurn = false;
                        deltaAngle = 360 - (currentAngle - goalAngle);
                        directTurn = false;
                    }
                }
                if (sideOfTurn)
                    setMotorPower(driveMotors, new double[][]{{power, -power}, {power, -power}});
                else
                    setMotorPower(driveMotors, new double[][]{{-power, power}, {-power, power}});
                if (directTurn)
                    while (Math.abs(angle0 - currentAngle) < deltaAngle) {  //motors running
                        currentAngle = getCurrentScaledAngle(imu);
                        telemetry.addData("angle case 3:", currentAngle);
                        telemetry.update();
                    }
                else if (goalAngle > 180 && currentAngle < 180)
                    while ((currentAngle <= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle > 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                        currentAngle = getCurrentScaledAngle(imu);
                        telemetry.addData("angle case 1:", currentAngle);
                        telemetry.update();
                    }

                else if (goalAngle < 180 && currentAngle > 180)
                    while ((currentAngle >= 180 && Math.abs(angle0 - currentAngle) < deltaAngle) || (currentAngle < 180 && 360 - Math.abs((angle0 - currentAngle)) < deltaAngle)) {//motors running
                        currentAngle = getCurrentScaledAngle(imu);
                        telemetry.addData("angle case 2:", currentAngle);
                        telemetry.update();
                    }


            setMotorPower(driveMotors, new double[][]{{0, 0}, {0, 0}});
        }

        public static void setMotorPower(DcMotor[][] motors, double[][] powers) {
            for (int i = 0; i < motors.length; i++)
                for (int j = 0; j < motors[i].length; j++)
                    motors[i][j].setPower(powers[i][j]);
        }

        public static void set2MotorPower(DcMotor[] motors, double powers) {
            for (int i = 0; i < motors.length; i++)
                motors[i].setPower(powers);
        }

        public static double getCurrentScaledAngle(BNO055IMU imu) {
            double angle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if (angle < 0)
                angle += 360;
            return angle;
        }

    }
}
