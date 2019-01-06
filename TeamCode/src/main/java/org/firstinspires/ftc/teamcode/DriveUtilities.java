package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveUtilities {
    public static void setMotorPower(DcMotor[][] driveTrain, double[][] power) { //Stores the four drivetrain motors power in array
        for (int row = 0;  row < driveTrain.length; row++)
            for (int col = 0;  col<driveTrain[row].length ; col++)
                driveTrain[row][col].setPower(power[row][col]);
    }}
