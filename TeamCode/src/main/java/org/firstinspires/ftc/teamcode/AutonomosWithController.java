package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "AutonomosWithController")
public class AutonomosWithController extends AutoMode{
    AutoMode auto;
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        while (opModeIsActive()) {
            if (gamepad1.a) {
                auto.LandInAuto(0.7, 0.7);
            }
            if (gamepad1.b) {
                auto.getCube();
            }
            if (gamepad1.x) {
                auto.Marker(0.7, -2800);

            }

            if (gamepad1.y) {
                auto.Parking(-500, 0.7, -750, 0.7);
            }
        }
    }
}
