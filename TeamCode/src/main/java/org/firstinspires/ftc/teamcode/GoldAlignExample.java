///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//@TeleOp(name = "GoldAlign Example", group = "DogeCV")
//
//public class GoldAlignExample extends OpMode {
//    private ElapsedTime runtime = new ElapsedTime();
//    DcMotor[][] drivetrainDC = new DcMotor[3][3];
//    double[][] sticks = new double[4][4];
//    int pos[][] = new int[2][2];
//    private GoldAlignDetector detector;
//
//
//    @Override
//    public void init() {
////        drivetrainDC[1][1] = hardwareMap.get(DcMotor.class, "rightFront");
////        drivetrainDC[0][1] = hardwareMap.get(DcMotor.class, "rightBack");
////        drivetrainDC[1][0] = hardwareMap.get(DcMotor.class, "leftFront");
////        drivetrainDC[0][0] = hardwareMap.get(DcMotor.class, "leftBack");
////
////
////        drivetrainDC[0][0].setDirection(DcMotorSimple.Direction.FORWARD);
////        drivetrainDC[0][1].setDirection(DcMotorSimple.Direction.REVERSE);
////        drivetrainDC[1][0].setDirection(DcMotorSimple.Direction.FORWARD);
////        drivetrainDC[1][1].setDirection(DcMotorSimple.Direction.REVERSE);
////
////        drivetrainDC[1][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        drivetrainDC[0][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        drivetrainDC[1][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        drivetrainDC[0][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");
//
//        detector = new GoldAlignDetector();
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
//        detector.useDefaults();
//
//        // Optional Tuning
//        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
//        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005;
//
//        detector.ratioScorer.weight = 5;
//        detector.ratioScorer.perfectRatio = 1.0;
//
//        detector.enable();
//
//
//    }
//
//    @Override
//    public void init_loop() {
//
//
//    }
//
//    /*
//     * Code to run ONCE when the driver hits PLAY
//     */
//    @Override
//    public void start() {
//
//    }
//
//
//    @Override
//    public void loop() {
//        if (!detector.isFound()){
//            telemetry.addLine("false!!!!!!!!!!!!!");
//        }
////        if (!detector.getAligned()) {
////            drivetrainDC[1][1].setPower(0.2);
////            drivetrainDC[0][1].setPower(0.2);
////            drivetrainDC[1][0].setPower(0.2);
////            drivetrainDC[0][0].setPower(0.2);
////        }
//        telemetry.addData("alignSize", detector.alignSize);
//        telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
//        telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos.
//        telemetry.update();
//    }
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//        detector.disable();
//    }
//
//}
//
