package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ParkingTest")
@Disabled
public class ParkingTest extends AutoMode {


    public void Parking(int targetPositionEncoder) {
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveUtils.driveByEncoderRoverRuckus(160, 160, 0.5, false);
        robot.shaft[0].setTargetPosition(targetPositionEncoder);//250
        robot.shaft[1].setTargetPosition(targetPositionEncoder);

        robot.shaft[0].setTargetPosition(750);
        robot.shaft[1].setTargetPosition(750);
        robot.shaft[0].setPower(0.6);
        robot.shaft[1].setPower(0.6);
        while (robot.shaft[0].isBusy() && robot.shaft[1].isBusy()) ;
            robot.linear.setTargetPosition(-750);
            while (robot.linear.isBusy());


    }


    @Override

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        driveUtils = new DriveUtilities(this);

        waitForStart();
        Parking(3000);

    }
}
