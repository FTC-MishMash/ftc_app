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


@TeleOp(name = "DRIVEY", group = "Iterative Opmode")
//@Disabled
public class DRIVEY extends OpMode {
    double posRelic;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor[][] drivetrainDC = new DcMotor[3][3];
    DcMotor[] intakeDC = new DcMotor[2];
    DcMotor relicDC;
    double[][] sticks = new double[4][4];
    Servo[] servosGlyph = new Servo[2];
    Servo servoSensor;
    Servo servoSide;
    //  Servo bringNew;
    DigitalChannel magnetDown;
    Servo servoArm;
    Servo[] relicServo = new Servo[2];
    ModernRoboticsI2cRangeSensor range;
    ColorSensor sensorColor;
    AnalogInput pot;
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
    int relicPos;    /*

     * Code to run ONCE when the driver hits INIT
     */

    @Override

    public void init() {

        telemetry.addData("Status", "Initialized");

        drivetrainDC[0][1] = hardwareMap.get(DcMotor.class, "right1");
        drivetrainDC[1][1] = hardwareMap.get(DcMotor.class, "right2");
        drivetrainDC[0][0] = hardwareMap.get(DcMotor.class, "left1");
        drivetrainDC[1][0] = hardwareMap.get(DcMotor.class, "left2");
        intakeDC[0] = hardwareMap.get(DcMotor.class, "g1");
        intakeDC[1] = hardwareMap.get(DcMotor.class, "g2");
        relicDC = hardwareMap.get(DcMotor.class, "dcRelic");
        servoSensor = hardwareMap.get(Servo.class, "servoSensor");
        servoArm = hardwareMap.get(Servo.class, "servoArm");

        magnetDown = hardwareMap.get(DigitalChannel.class, "magnetDown");
        servosGlyph[0] = hardwareMap.get(Servo.class, "servoBring");
        servoSide = hardwareMap.get(Servo.class, "servoSides");
        servosGlyph[1] = hardwareMap.get(Servo.class, "servoBring2");
        relicServo[0] = hardwareMap.get(Servo.class, "relicServo1");
        relicServo[1] = hardwareMap.get(Servo.class, "relicServo2");
        // relicServo[1].scaleRange(0, 1);
        pot = hardwareMap.get(AnalogInput.class, "pot");
        touchUp = hardwareMap.get(DigitalChannel.class, "TouchUp");
        touchDown = hardwareMap.get(DigitalChannel.class, "TouchDown");
        for (int i = 0; i < 1; i++) {
            // foor (int i = 0; i < 2; i++) {

            for (int j = 0; j < 1; j++) {
                //     for (int j = 0; j < 2; j++) {


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
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("servoBring", servosGlyph[0].getPosition());

        telemetry.update();

    }

       /*
        * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
        */


    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            servoSensor.setPosition(0.8);
        } else if (gamepad1.dpad_down) {
            servoSensor.setPosition(0.15);

        }
        telemetry.addData("magnet", magnetDown.getState());
        telemetry.addData("up ", touchUp.getState());
        telemetry.addData("down ", touchDown.getState());
        //        telemetry.update();
        up = touchUp.getState();
        down = touchDown.getState();
//        telemetry.addData("SwitchUp", up);
//        telemetry.addData("SwitchDown", down);
//        telemetry.addData("relic 1", posRelic);

        if (gamepad1.b)
            a = 2;
        else if (gamepad1.a)
            a = 1;
        telemetry.addData("a: ", a);
//        telemetry.update();
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

        /*if (gamepad1.y)
            gain = 0.6;
        if (gamepad1.x)
            gain = 1;
*/

        if (gamepad1.right_trigger != 0)//strafe to the right
            strafe(1);

        else if (gamepad1.left_trigger != 0)//strafe to the left
            strafe(-1);
        else
            tankDriveTrainSetPower(gain);

        if (a == 1) {
            if (relicStop)
                relicServo[1].setPosition(0);
            relicDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addLine("GlyphMode");
            telemetry.update();
            if (gamepad2.right_bumper)// intake
                intakeOperation(-1);
            else if (gamepad2.right_trigger != 0)// backwards intake
                intakeOperation(1);
            else
                intakeOperation(0);
            // GLYPH DOOR MOVING
            // CHECK IF DOOR IS UP
            if (touchUp.getState() && !gamepad2.y) {
                flagDoorMovingUp = false;
                flagSideOpen = true;
            } else
                flagDoorMovingUp = true;
            // CHECK IF DOOR IS DOWN
            if ((touchDown.getState() || !magnetDown.getState()) && !gamepad2.x)
            {
                flagDoorMovingDown = false;
                flagSideOpen = true;
            }
            else
                flagDoorMovingDown = true;
            // OPERATOR PRESSED KEY TO MOVE DOOR
            if (gamepad2.x || gamepad2.y)
            {
                // CLOSE SIDE HOLDERS
                if (flagSideOpen && !flagSideWait && !finishDoor)
                {
                    servoSide.setPosition(0);
                    tSideWait = getRuntime();
                    flagSideWait = true;
                }


                // WAIT FOR SIDE HOLDERS TO CLOSE
                if (flagSideWait) {
                    //  telemetry.addData("getRuntime()", runtime.seconds());

                    if (gamepad2.x && getRuntime() - tSideWait > 0.4)
                    {
                        flagSideWait = false;
                        flagSideOpen = false;
                    } else if (gamepad2.y) {
                        flagSideWait = false;
                        flagSideOpen = false;
                    }
                }

                // DOOR GOING UP
                if (!flagSideOpen && flagDoorMovingUp && gamepad2.x) {
                    servosGlyph[0].setPosition(0);
                    servosGlyph[1].setPosition(1);
                }
                // DOOR GOING DOWN
                else if (!flagSideOpen && flagDoorMovingDown && gamepad2.y) {
                    servosGlyph[0].setPosition(1);
                    servosGlyph[1].setPosition(0);
                    finishDoor = false;
                }
                // KEY PRESSED BY OPERATOR BUT DOOR IS ALREADY UP OR DOWN
                else {
                    servosGlyph[0].setPosition(0.5);
                    servosGlyph[1].setPosition(0.5);
                }
                if (gamepad2.y && !flagDoorMovingDown && !finishDoor) {
                    servoSide.setPosition(0.9);
                    flagSideOpen = true;
                    finishDoor = true;
                }
            }
            // OPERATOR DID NOT NOT NOT NOT PRESS KEY TO MOVE DOOR
            else if (gamepad2.b) {
                servosGlyph[0].setPosition(1);
                servosGlyph[1].setPosition(0);
            } else if (gamepad2.a) {
                servosGlyph[0].setPosition(0);
                servosGlyph[1].setPosition(1);
            } else {
                servosGlyph[0].setPosition(0.5);
                servosGlyph[1].setPosition(0.5);
                finishDoor = false;
            }

            // DRIVER OPENS SIDE HOLDERS
            if (gamepad1.right_bumper) {
                servoSide.setPosition(0.4);
                flagSideOpen = true;
            }
            // DRIVER CLOSES SIDE HOLDERS
            if (gamepad1.left_bumper) {
                servoSide.setPosition(0);
                flagSideOpen = false;
            }
            // OPERATOR OPENS SIDE HOLDERS
            if (gamepad2.dpad_up && (!flagDoorMovingUp || !flagDoorMovingDown)) {
                servoSide.setPosition(1);
                flagSideOpen = true;
            }
            // OPERATOR CLOSES SIDE HOLDERS
            if (gamepad2.dpad_down && (!flagDoorMovingUp || !flagDoorMovingDown))
            {
                servoSide.setPosition(0);
                flagSideOpen = false;
            }


            if (gamepad2.left_trigger != 0) {
                intakeDC[0].setPower(1);
                intakeDC[1].setPower(-1);
            }
            if (touchUp.getState() && !gamepad2.y && !gamepad2.a && !gamepad2.b) {
                servosGlyph[0].setPosition(0.5);
                servosGlyph[1].setPosition(0.5);
            }

        } else if (a == 2) {
            relicStop = true;
            telemetry.addLine("RELIC mode");
            telemetry.update();
            servosGlyph[0].setPosition(0.5);
            servosGlyph[1].setPosition(0.5);
            intakeOperation(0);
            if (gamepad1.right_bumper)// intake
                intakeOperation(-1);
            else if (gamepad1.left_bumper)// backwards intake
                intakeOperation(1);
//            else if (gamepad1.right_bumper) {//crosing
//                intakeDC[0].setPower(0.7);
//                intakeDC[1].setPower(-0.7);
//            }
            else
                intakeOperation(0);

            relicEncoder = relicDC.getCurrentPosition();
            if (count == 0) {
                relicDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                count++;
            }
            relicPos = relicDC.getCurrentPosition();
            //            telemetry.addLine("RelicMode");
            //            telemetry.update();
            if (gamepad2.dpad_up)//opens the relic arm
            {
                relicDC.setTargetPosition(relicPos - 80);
                relicDC.setPower(1);
                relicDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad2.dpad_down) {
                relicDC.setTargetPosition(relicPos + 80);
                relicDC.setPower(1);
                relicDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else
                relicDC.setPower(0.9);

            if (gamepad2.a) {
                relicServo[0].setPosition(0.3);

            }
            if (gamepad2.b) {
                relicServo[0].setPosition(0);//close
            }
            if (gamepad1.dpad_left)
                relicServo[1].setPosition(1);
            if (gamepad1.dpad_right)
                relicServo[1].setPosition(0.1);
            else if (gamepad2.right_trigger != 0) {
                // posRelic -= 0.03;
                relicServo[1].setPosition(posRelic);
                telemetry.addData("relic 1", posRelic);
                //                telemetry.update();
            }

            if (gamepad2.left_bumper) {
                // posServo += .01;
                relicServo[1].setPosition(0.1);
            }
            if (gamepad2.right_bumper) {
                relicServo[1].setPosition(1);
            }
            if (gamepad2.y) {
                if (relicEncoder > 1200) {
                    relicDC.setTargetPosition(relicEncoder - 500);
                    relicDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    relicDC.setPower(1);
                    gain = 1;
                }

                //                if (relicDC.getCurrentPosition() < 130) {
                //                }
            }
            if (gamepad2.x) {
                if (relicEncoder < 2500) {
                    relicDC.setTargetPosition(relicEncoder + 500);
                    relicDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    relicDC.setPower(1);
                }
            }

//            if (gamepad2.left_trigger != 0) {
//                if (relicEncoder > 90) {
//                    relicDC.setTargetPosition(relicEncoder - 200);
//                    relicDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    relicDC.setPower(1);
//                }
//                //                if (relicDC.getCurrentPosition() < 130) {
//                relicServo[0].setPosition(0);
//                relicServo[1].setPosition(1);
//                //                }
//            }
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
                relicServo[0].setPosition(0.5);
                relicServo[1].setPosition(1);
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
                relicServo[0].setPosition(0);
                relicServo[1].setPosition(1);

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
        servosGlyph[index].setPosition(position);
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

