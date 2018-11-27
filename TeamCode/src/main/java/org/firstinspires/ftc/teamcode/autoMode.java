package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by user on 22/11/2018.
 */
@Autonomous(name = "gyroBalance")
public class autoMode extends LinearOpMode {
    Robot robot;
    DcMotor[] motorsHanging;
    ElapsedTime runTime = new ElapsedTime();
    BNO055IMU imu;
    static final int PitchtargetAngleMin = -5;
    static final int PitchtargetAngleMax = 5;
    static final int RolltargetAngleMin = -10;
    static final int RolltargetAngleMax = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        waitForStart();
        runTime.startTime();
        runTime.reset();
        imu = robot.getImu();
        motorsHanging=robot.shaft;
        double power=0.6;
        motorsHanging[0].setPower(power);
        motorsHanging[1].setPower(power);
        while (opModeIsActive()) {
            if (getOffTheClimb()) {
                telemetry.addLine("got balanced");
                telemetry.update();
                break;
            } else {
                telemetry.addLine("not balanced");
                telemetry.update();
            }
            motorsHanging[0].setPower(0);
            motorsHanging[1].setPower(0);
        }
    }

    public boolean getOffTheClimb() {
        Orientation axis = getAxis();
        return axis.secondAngle > RolltargetAngleMin && axis.secondAngle < RolltargetAngleMax && axis.thirdAngle > PitchtargetAngleMin && axis.thirdAngle < PitchtargetAngleMax;

    }

    public Orientation getAxis() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
