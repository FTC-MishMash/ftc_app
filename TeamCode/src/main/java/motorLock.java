import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class motorLock extends LinearOpMode {
    Robot robot;

    DcMotor[] linear = new DcMotor[2];
    DcMotor[] shaft = new DcMotor[2];


    @Override
    public void runOpMode() throws InterruptedException {
        linear[0] = hardwareMap.get(DcMotor.class, "linearLeft");
        shaft[0] = hardwareMap.get(DcMotor.class, "shaftRight");
        shaft[1] = hardwareMap.get(DcMotor.class, "shaftLeft");

        shaft[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shaft[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linear[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shaft[0].setTargetPosition(0);
        shaft[1].setTargetPosition(0);
        linear[0].setTargetPosition(0);

        shaft[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shaft[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        shaft[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shaft[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shaft[0].setPower(1);
        shaft[1].setPower(1);
        linear[0].setPower(1);

        waitForStart();
        MotorsLock();

    }

    public void MotorsLock() {

        while (opModeIsActive()) {


            shaft[0].setTargetPosition(0);
            shaft[1].setTargetPosition(0);
            linear[0].setTargetPosition(0);
            shaft[0].setPower(1);
            shaft[1].setPower(1);
            linear[0].setPower(1);
            telemetry.addData("1: ", shaft[0].getCurrentPosition());
            telemetry.addData("2: ", shaft[1].getCurrentPosition());
            telemetry.addData("3: ", linear[0].getCurrentPosition());
            telemetry.update();
        }
        shaft[0].setPower(0);
        shaft[1].setPower(0);
        linear[0].setPower(0);
    }
}