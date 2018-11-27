package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by user on 06/11/2018.
 */
@Autonomous(name = "ImageTesting")
public class navigationToTargert extends LinearOpMode {
    Robot robot;
    DcMotor[][] motors;
    static final int XtargetPosition=-65;
    static final int YtargetPosition=58;
    static final int ZtargetPosition=-2;
    static final int PitchtargetAngleMin=-90;
    static final int PitchtargetAngleMax=-80;
    static final int HeadingtargetAngleMin=20;
    static final int HeadingtargetAngleMax=40;
    static final int RolltargetAngleMin=0;
    static final int RolltargetAngleMax=0;



    @Override
    public void runOpMode() throws InterruptedException {
//        robot = new Robot(hardwareMap);
//        waitForStart();
//        motors = robot.getDriveTrain();
        waitForStart();
        VuforiaWebCamImagesTargets.initVuforia(hardwareMap);
        if (opModeIsActive()) {
            driveToImage();

        }
    }

    /**
     * Driving the robot near the target image and turn it across the depot.
     */
    private void driveToImage() {
        Driving.setMotorPower(motors, new double[][]{{0.5, 0.5}, {0.5, 0.5}});
        float[] positions = VuforiaWebCamImagesTargets.getPositions();
        while (positions[0] < XtargetPosition || positions[1] > YtargetPosition|| positions[2] > ZtargetPosition || positions[3] > PitchtargetAngleMin || positions[3] < PitchtargetAngleMax || positions[5] < HeadingtargetAngleMin|| positions[5] > HeadingtargetAngleMax)
        positions = VuforiaWebCamImagesTargets.getPositions();
      //  Driving.ScaledTurn(90,motors,robot.imu,0.6,telemetry);

    }
}
