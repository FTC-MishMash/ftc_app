import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autoMode;

public class motorLock {
    Robot robot;
    autoMode auto;

    public void MotorsLock() {

        int startCurrentPosision[][] = new int[2][2];
        startCurrentPosision[0][0] = robot.shaft[0].getCurrentPosition();
        startCurrentPosision[1][0] = robot.shaft[1].getCurrentPosition();
        startCurrentPosision[0][1] = robot.linear[0].getCurrentPosition();


        robot.shaft[0].setTargetPosition(robot.driveTrain[0][0].getCurrentPosition());
        robot.shaft[1].setTargetPosition(robot.driveTrain[1][0].getCurrentPosition());
        robot.linear[0].setTargetPosition(robot.driveTrain[0][1].getCurrentPosition());


        robot.shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}
