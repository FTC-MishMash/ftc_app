package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by user on 06/11/2018.
 */

public class Robot
{
    public DcMotor[][] driveTrain;

    public Robot(HardwareMap hardwareMap){
        driveTrain=new DcMotor[2][2];
        driveTrain[0][0]=hardwareMap.get(DcMotor.class,"leftFront");
        driveTrain[1][0]=hardwareMap.get(DcMotor.class,"leftBack");
        driveTrain[0][1]=hardwareMap.get(DcMotor.class,"rightFront");
        driveTrain[1][1]=hardwareMap.get(DcMotor.class,"rightBack");

    }
}
