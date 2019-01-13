package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

@Autonomous(name = "VuforiaCheck")
public class VuforiaCheck extends AutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        tsSampling.initVuforiaWebCam(true);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tsSampling.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        //motorLock();
        waitForStart();
        //LandInAuto();


        while (opModeIsActive()) {
            if (vuforia == null)
                telemetry.addLine("null");
            if (gamepad1.x) {
                vuforia.close();
                VuforiaLocalizerImpl.CloseableFrame frame=vuforia.getFrameQueue().take();
                frame.close();
                telemetry.addLine("closed");

            }
            if (gamepad1.a)
                tsSampling.initVuforiaWebCam(false);
            telemetry.update();

        }
    }
}

