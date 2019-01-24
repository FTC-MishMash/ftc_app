package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ParkingTest")

public class ParkingTest extends AutoMode {

    Robot robot;
    DriveUtilities ds;

    public void Parking(int targetPositionEncoder) {
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ds.driveByEncoderRoverRuckus(90, 90, 0.5, false);
        robot.shaft[0].setTargetPosition(targetPositionEncoder);//250
        robot.shaft[1].setTargetPosition(targetPositionEncoder);

        robot.shaft[0].setPower(0.6);
        robot.shaft[1].setPower(0.6);
    }

    @Override

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        waitForStart();
        Parking(3000);

    }
}
