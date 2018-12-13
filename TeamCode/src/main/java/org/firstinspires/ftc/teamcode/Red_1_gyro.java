package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous
public class Red_1_gyro extends LinearOpMode {
    Robot robot;
    DcMotor[][] DriveMotors;

    public void gyro_drive() {
        double power = 0.3;
        setMotorPower(new double[][]{{power, power}, {power, power}});
        while (opModeIsActive() && getRoll() >= -30) {
            setMotorPower(new double[][]{{power, power}, {power, power}});
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot=new Robot(hardwareMap);
        robot.imu = robot.getImu();
        DriveMotors = robot.getDriveTrain();
        waitForStart();
        gyro_drive();


    }

    public float getRoll() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
    }

    public void setMotorPower(double[][] power) { //Stores the four drivetrain motors power in array
        for (int row = 0; opModeIsActive() && row < 2; row++)
            for (int col = 0; opModeIsActive() && col < 2; col++)
                robot.driveTrain[row][col].setPower(power[row][col]);
    }

}
