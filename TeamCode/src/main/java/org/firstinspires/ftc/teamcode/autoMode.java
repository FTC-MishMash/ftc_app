package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by user on 22/11/2018.
 */

public class autoMode extends LinearOpMode {
    Robot robot;
    DcMotor[][] motors;
    ElapsedTime runTime = new ElapsedTime();
    BNO055IMU imu;
    static final int PitchtargetAngleMin=-5;
    static final int PitchtargetAngleMax=5;
    static final int RolltargetAngleMin=-10;
    static final int RolltargetAngleMax=10;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        waitForStart();
        runTime.startTime();
        runTime.reset();
        imu=robot.getImu();
        motors = robot.getDriveTrain();
        if (opModeIsActive()) {

        }
    }
public                              
    public boolean GetOffTheClimb() {
        return imu.getAngularOrientation().secondAngle>angle1&&imu.getAngularOrientation().thirdAngle>angle2;
    }
}
