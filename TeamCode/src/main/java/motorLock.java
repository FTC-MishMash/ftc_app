import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autoMode;
@Autonomous
public class motorLock extends LinearOpMode {
    Robot robot;


    @Override
    public void runOpMode()throws InterruptedException {
        waitForStart();
        MotorsLock();

    }
    public void MotorsLock() {

//
        robot.shaft[0].setTargetPosition(robot.shaft[0].getCurrentPosition());
        robot.shaft[1].setTargetPosition(robot.shaft[1].getCurrentPosition());
        robot.linear[0].setTargetPosition(robot.linear[0].getCurrentPosition());


        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}