package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class ParkInCrator extends LinearOpMode {
    Robot robot;
    autoMode auto = new autoMode();
    BNO055IMU imu;
    /**
     * @param heading
     * used to control the robot orientaion while driving.
     */


    /**
     * used to set the speed of the robot while driving and to use PID to control the robot turns.
     */
    public void driveToCrater() {
        double power = 0.7;
        double pidErr[] = {0, 0};

        while (opModeIsActive() && getAngulerOriantion().thirdAngle >= 12.8) {
            pidErr = auto.GyroPID(getAngulerOriantion().thirdAngle, pidErr[1], robot.imu);
            auto.setMotorPower(new double[][]{{power + pidErr[0], power + pidErr[0]}, {power - pidErr[0], power - pidErr[0]}});

        }

        auto.setMotorPower(new double[][]{{0, 0}, {0, 0}});

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        imu = robot.getImu();
        waitForStart();
        driveToCrater();

    }


    public Orientation getAngulerOriantion() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    }

//    public void setMotorPower(double[][] power) { //Stores the four drivetrain motors power in array
//        for (int row = 0; opModeIsActive() && row < 2; row++)
//            for (int col = 0; opModeIsActive() && col < 2; col++)
//                robot.driveTrain[row][col].setPower(power[row][col]);
//    }

}
