package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class LandInAuto extends LinearOpMode {
    Robot robot;
    autoMode auto;


    public void LandInAuto() {
        while (opModeIsActive() && getAngularOriention().thirdAngle <= 0) {
            robot.linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linear.setTargetPosition(0);
            robot.linear.setPower(1);

            robot.shaft[0].setPower(0.3);
            robot.shaft[1].setPower(0.3);
        }
        robot.shaft[0].setPower(0);
        robot.shaft[1].setPower(0);

        while (opModeIsActive() && robot.linear.getCurrentPosition()<=600){
            robot.linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.linear.setTargetPosition(1200);
            robot.linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linear.setPower(0.5);

        }
robot.linear.setPower(0);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        waitForStart();
        LandInAuto();

    }

    private Orientation getAngularOriention() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }



}
