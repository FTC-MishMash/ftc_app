package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import java.util.List;

public class TensorflowUtils {
    AutoMode currOpMode;
    static int count = 0;
    public VuforiaLocalizerEx vuforia;
    Robot robot;
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    //    private static final android.graphics.Color Color = ;
    public static final String VUFORIA_KEY = " ATgDONj/////AAABmW0G/nQirUMiumnzPc6Pl8oJhBOCC2qoUq0BWhir9YWcBFDlhZUfSwATcQArcyyLxIOV21sHaYJeeQEJZfIJ+4spBn3oJ/DfycsbPaNs87+TRpM46/vbUkj1Ok+NtZ/eqMhmMXjFC8dgdCfbCt0aMxoBNzDw4+v28abG+hjUCjVYf86Jq1m7R942XCjw0yhOZqTXWIp3WAZDXY/PdWGQGY/zWae0l6TAZ6Z27t1xYJdkkpLqEsbKM3ZprvtgIs8AsWS9Tri2892OHq2CnCL+1ZHHXKPdxON3fiC1Gd3oihwPhTUReNw0VAg9yeVsVa1UQg7ea9K6WpmVto0FG+T2/LV8uq/3Mp/NHWiNizw2DM4h";
    Telemetry telemetry;
    DriveUtilities driveUtilities;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    public TensorflowUtils(AutoMode currOpMode) {
        this.currOpMode = currOpMode;
        this.vuforia = currOpMode.vuforia;
        this.tfod = currOpMode.tfod;
        this.robot = currOpMode.robot;
        this.telemetry = currOpMode.telemetry;
        this.driveUtilities = currOpMode.driveUtils;
    }

    public void initTfod() {

        int tfodMonitorViewId = currOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", currOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        tfodParameters.minimumConfidence=0.25;
        currOpMode.tfod = tfod;
    }

    public void initVuforiaWebCam(boolean webcam) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters;
        int cameraId = currOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", currOpMode.hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(/*cameraId*/);

//        if (count > 1)
//            parameters = new VuforiaLocalizer.Parameters(cameraId);
//else {
//            count++;
//            parameters = new VuforiaLocalizer.Parameters(cameraId);
//        }        //  Instantiate the Vuforia engine
        //vuforia = (VuforiaLocalizer) ClassFactory.getInstance().createVuforia(parameters);
        if (webcam) {
            WebcamName webcamName = currOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
            if (webcamName.isAttached())
                parameters.cameraName = webcamName;
        }
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforia = new VuforiaLocalizerEx(parameters);
        currOpMode.vuforia = this.vuforia;
        currOpMode.targetNav.vuforia = this.vuforia;
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    public void followCubeRecognision(double power) {
        double runTime = 0;
        telemetry.addLine("follow cube 1:");
        telemetry.update();

        double distanceFromRight = 0;
        double distanceFromLeft = 0;
        double middleCubeX = 0;
        double k = 0.0007; //EDEN
        double[] addToMotors;
        addToMotors = new double[2];

        boolean breakLoop = false;
        runTime = currOpMode.getRuntime();

//        RecognitionList.get(indexGold);
        if (tfod != null)
            do {
                List<Recognition> RecognitionList = tfod.getRecognitions();// I delete List<Recognition>
                Recognition goldReco = null;

                if (RecognitionList != null) {
                    for (Recognition recognition : RecognitionList) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldReco = recognition;
                            runTime = currOpMode.getRuntime();
                            break;
                        }
                    }
                }

                if (goldReco != null) {

                    middleCubeX = ((goldReco.getLeft() + goldReco.getRight()) / 2);
                    distanceFromRight = 720 - middleCubeX;
                    distanceFromLeft = middleCubeX;
                    telemetry.addLine("follow cube 4:");

                    addToMotors[0] = k * distanceFromRight;  //RIGHT
                    addToMotors[1] = k * distanceFromLeft; //LEFT
                    telemetry.addData("distance From Right:", addToMotors[0]);
                    telemetry.addData("distance From Left:", addToMotors[1]);
                    telemetry.update();


                    robot.driveTrain[0][1].setPower(addToMotors[0] + power);//RIGHT Front
                    robot.driveTrain[1][1].setPower(addToMotors[0] + power);//Right Back
                    robot.driveTrain[1][0].setPower(addToMotors[1] + power);//Left Back
                    robot.driveTrain[0][0].setPower(addToMotors[1] + power);//LEFT Front
                } else {
                    telemetry.addLine("gold reco = null");
                    telemetry.update();
                    robot.driveTrain[0][1].setPower(0);//RIGHT Front
                    robot.driveTrain[1][1].setPower(0);//Right Back
                    robot.driveTrain[1][0].setPower(0);//Left Back
                    robot.driveTrain[0][0].setPower(0);//LEFT Front
                    if ((runTime - currOpMode.getRuntime()) < -2) {

                        breakLoop = true;
                        telemetry.addLine("in 2 seconds dont see the cube");
                        telemetry.update();
                    }
                }


            }
            while (currOpMode.opModeIsActive() && !breakLoop);
        robot.driveTrain[0][1].setPower(0);//RIGHT Front
        robot.driveTrain[1][1].setPower(0);//Right Back
        robot.driveTrain[1][0].setPower(0);//Left Back
        robot.driveTrain[0][0].setPower(0);//LEFT Front
    }

    public int searchCube(double power, int turnAngleRight, int turnAngleLeft) {
        int cubePosition = 0;
        if (tfod == null) {
            telemetry.addData("tfod is NULL  ", tfod);
            telemetry.update();
            cubePosition = -1;
            return cubePosition;
        }
//        } else if (tfod != null) {
//            tfod.activate();
//        }

        double runTime0 = currOpMode.getRuntime();
        while (currOpMode.opModeIsActive() && currOpMode.getRuntime() - runTime0 < 3) {
            java.util.List<Recognition> RecognitionList = tfod.getUpdatedRecognitions();
            telemetry.addData("have cube?   ", RecognitionList != null);
            telemetry.update();

            if (RecognitionList != null)
                for (Recognition recognition : RecognitionList) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        cubePosition = 1;//CENTER
                        return cubePosition;
                    }
                }
        }
        //only if dont have cube in middle
        driveUtilities.TurnWithEncoder(turnAngleRight, power);
        runTime0 = currOpMode.getRuntime();
        while (currOpMode.opModeIsActive() && currOpMode.getRuntime() - runTime0 < 2) {
            java.util.List<Recognition> RecognitionList = tfod.getUpdatedRecognitions();
            telemetry.addLine("have cube?   ");
            telemetry.update();

            if (RecognitionList != null)//TODO: להבין למה לא הלך שמאלה למרות שהייתה קוביה בימין
                for (Recognition recognition : RecognitionList) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        cubePosition = 2;//RIGHT
                        return cubePosition;
                    }
                }
        }

        // cubePosition != 1 && cubePosition != 2
        cubePosition = 3;//LEFT
        driveUtilities.TurnWithEncoder(turnAngleLeft, power);//encoder=0.3
        runTime0 = currOpMode.getRuntime();
        while (currOpMode.opModeIsActive() && currOpMode.getRuntime() - runTime0 < 2) {
            List<Recognition> RecognitionList = tfod.getUpdatedRecognitions();
            telemetry.addLine("have cube?   ");
            telemetry.update();

            if (RecognitionList != null)//TODO: להבין למה לא הלך שמאלה למרות שהייתה קוביה בימין
                for (Recognition recognition : RecognitionList) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        cubePosition = 3;//RIGHT
                        return cubePosition;
                    }
                }
        }
        driveUtilities.TurnWithEncoder(0, power);
        return cubePosition;
    }

}