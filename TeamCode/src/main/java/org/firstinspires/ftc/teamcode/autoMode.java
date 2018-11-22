package org.firstinspires.ftc.teamcode;

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

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        waitForStart();
        runTime.startTime();
        runTime.reset();
        motors = robot.getDriveTrain();
        if (opModeIsActive()) {


        }
    }

}
