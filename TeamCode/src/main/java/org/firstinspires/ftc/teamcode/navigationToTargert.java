package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by user on 06/11/2018.
 */

public class navigationToTargert extends LinearOpMode {
Robot robot;
    DcMotor[][] motors;
    @Override
    public void runOpMode() throws InterruptedException {
        robot=new Robot(hardwareMap);
        waitForStart();
        motors=robot.getDriveTrain();
        if (opModeIsActive()){
            driveToImage();

            }
        }


    private void driveToImage() {
        Driving.setMotorPower(motors,new Double[][]{{0.5,0.5},{0.5,0.5}});
    }
}
