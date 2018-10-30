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


@TeleOp(name = "Turning", group = "Iterative Opmode")
//@Disabled
public class DriveTrain extends OpMode {
    //hello
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor[][] drivetrainDC = new DcMotor[3][3];
    double[][] sticks = new double[4][4];
    int pos[][] = new int[2][2];


    @Override

    public void init() {



        drivetrainDC[1][1] = hardwareMap.get(DcMotor.class, "rightFront");
        drivetrainDC[0][1] = hardwareMap.get(DcMotor.class, "rightBack");
        drivetrainDC[1][0] = hardwareMap.get(DcMotor.class, "leftFront");
        drivetrainDC[0][0] = hardwareMap.get(DcMotor.class, "leftBack");



        drivetrainDC[0][0].setDirection(DcMotorSimple.Direction.FORWARD);
        drivetrainDC[0][1].setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrainDC[1][0].setDirection(DcMotorSimple.Direction.FORWARD);
        drivetrainDC[1][1].setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrainDC[1][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrainDC[0][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrainDC[1][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrainDC[0][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }

    @Override
    public void init_loop() {

    }



    @Override
    public void start() {
        runtime.reset();


        telemetry.update();

    }


    @Override
    public void loop() {

        sticks[0][0] = -gamepad1.left_stick_x;
        sticks[1][0] = -gamepad1.left_stick_y;
        sticks[0][1] = -gamepad1.right_stick_x;
        sticks[1][1] = -gamepad1.right_stick_y;


        tankDriveTrainSetPower(1);

    }


    void tankDriveTrainSetPower(double gain) {
        drivetrainDC[0][0].setPower(gain * (gamepad1.left_stick_y));
        drivetrainDC[1][0].setPower(gain * (gamepad1.left_stick_y));
        drivetrainDC[0][1].setPower(gain * (gamepad1.right_stick_y));
        drivetrainDC[1][1].setPower(gain * (gamepad1.right_stick_y));
    }






}