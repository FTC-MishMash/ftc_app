package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class LandInAuto extends LinearOpMode {
    Robot robot;
    autoMode auto;

    /**
     * @param heading
     * used to control the robot orientaion while driving.
     */
//    private double[] GyroPID(double heading, double lasterror, BNO055IMU imu) {
//        double kp = 0.015, kd = 0.01, ki = 0, nexterror = 0;
//        double err = heading - imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        while (err > 180)
//            err = err - 360;
//        while (err < -180)
//            err = err +  360;
//        lasterror = err - lasterror;
//        double pd = nexterror * ki + lasterror * kd + err * kp;
//        return (new double[]{-pd, err});
//    }

    /**
     * used to set the speed of the robot while driving and to use PID to control the robot turns.
     */
    public void driveToCrater() {
        double power = 0.7;
        double pidErr[] = {0, 0};

        while (opModeIsActive() && getAngularOriantion().thirdAngle >= -14.3) {
            pidErr = auto.GyroPID(getAngularOriantion().secondAngle, pidErr[1], robot.imu);
            auto.setMotorPower(new double[][]{{power + pidErr[0], power + pidErr[0]}, {power - pidErr[0], power - pidErr[0]}});

        }

        auto.setMotorPower(new double[][]{{0, 0}, {0, 0}});

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.imu = robot.getImu();
        waitForStart();
        driveToCrater();

    }

    private Orientation getAngularOriantion() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }



//    public void setMotorPower(double[][] power) { //Stores the four drivetrain motors power in array
//        for (int row = 0; opModeIsActive() && row < 2; row++)
//            for (int col = 0; opModeIsActive() && col < 2; col++)
//                robot.driveTrain[row][col].setPower(power[row][col]);
//    }

}
