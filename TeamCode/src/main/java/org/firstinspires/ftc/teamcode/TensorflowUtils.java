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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
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
    public static boolean isWebcamActivate = false;
    Telemetry telemetry;
    DriveUtilities driveUtilities;

    public static enum GOLD_MINERAL_POSITION {
        LEFT,
        CENTER,
        RIGHT,
        NONE;
    }

    enum MINERAL_HALF {
        RIGHT_HALF,
        LEFT_HALF;
    }

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */

    public TFObjectDetector tfod;

    public Recognition[] getSampling(List<Recognition> updateRecognitions) {
        Recognition[] twoSmallestMinerals = new Recognition[2];
        Recognition temp;
        if (updateRecognitions != null && updateRecognitions.size() >= 1) {
            twoSmallestMinerals[0] = updateRecognitions.get(0);
            if (updateRecognitions.size() >= 2) {
                twoSmallestMinerals[1] = updateRecognitions.get(1);
                for (Recognition recognition : updateRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) || recognition.getConfidence() > 0.4) {
                        if (recognition.getTop() < twoSmallestMinerals[0].getTop()) {
                            temp = twoSmallestMinerals[0];
                            twoSmallestMinerals[0] = recognition;
                            twoSmallestMinerals[1] = temp;
                        } else if (updateRecognitions.indexOf(recognition)!=0&&recognition.getTop() < twoSmallestMinerals[1].getTop()) {
                            twoSmallestMinerals[1] = recognition;
                        }
                    }
                }
            }
        }
        return twoSmallestMinerals;
    }


    public GOLD_MINERAL_POSITION goldPosition() {
        GOLD_MINERAL_POSITION cubePosition = GOLD_MINERAL_POSITION.NONE;

        if (tfod != null) {
            List<Recognition> updatetedRecognitions = tfod.getUpdatedRecognitions();
            Recognition[] samplingMinerals = getSampling(updatetedRecognitions);
            if (samplingMinerals[0] != null) {
                if (samplingMinerals[0].getLabel().equals(LABEL_GOLD_MINERAL)) {
                    if (samplingMinerals[0].getLeft() < samplingMinerals[0].getRight())
                        cubePosition = GOLD_MINERAL_POSITION.CENTER;
                    else
                        cubePosition = GOLD_MINERAL_POSITION.RIGHT;
                }

                if (samplingMinerals[0].getLabel().equals(LABEL_SILVER_MINERAL))
                    if (samplingMinerals[1] == null || (samplingMinerals[1] != null && samplingMinerals[1].getLabel().equals(LABEL_SILVER_MINERAL)))
                        cubePosition = GOLD_MINERAL_POSITION.LEFT;
                    else if (samplingMinerals[1].getLabel().equals(LABEL_GOLD_MINERAL)
                            && mineralPosition(samplingMinerals[0]) == mineralPosition(samplingMinerals[1])) {
                        cubePosition = GOLD_MINERAL_POSITION.LEFT;
                    } else {
                        if (samplingMinerals[1].getLeft() > samplingMinerals[1].getRight())
                            cubePosition = GOLD_MINERAL_POSITION.RIGHT;
                        else
                            cubePosition = GOLD_MINERAL_POSITION.LEFT;
                    }

            }
        }


        return cubePosition;
    }

    public MINERAL_HALF mineralPosition(Recognition samplingMineral) {
        double center = 0.5 * (samplingMineral.getLeft() + samplingMineral.getRight());
        if (samplingMineral.getLeft() > 400)
            return MINERAL_HALF.LEFT_HALF;
        return MINERAL_HALF.RIGHT_HALF;
    }

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
        tfodParameters.minimumConfidence = 0.11;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        currOpMode.tfod = tfod;
    }

    public void initVuforia(boolean webcam) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters;
        int cameraId = currOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", currOpMode.hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters();

//        if (count > 1)
//            parameters = new VuforiaLocalizer.Parameters(cameraId);
//else {
//            count++;
//            parameters = new VuforiaLocalizer.Parameters(cameraId);
//        }        //  Instantiate the Vuforia engine
        //vuforia = (VuforiaLocalizer) ClassFactory.getInstance().createVuforia(parameters);
        if (webcam) {
            WebcamName webcamName = currOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
            if (webcamName.isAttached()) {
                parameters.cameraName = webcamName;
                isWebcamActivate = true;
            }
        }
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforia = new VuforiaLocalizerEx(parameters);
        currOpMode.vuforia = this.vuforia;
        currOpMode.targetNav.vuforia = this.vuforia;
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    public void rotateToCube(double power, int turnAngleRight, int turnAngleLeft, GOLD_MINERAL_POSITION goldMineralPosition) {


        double runTime0 = currOpMode.getRuntime();
        if (goldMineralPosition == GOLD_MINERAL_POSITION.LEFT)
            driveUtilities.Turn(turnAngleLeft);
        else if (goldMineralPosition == GOLD_MINERAL_POSITION.RIGHT)
            driveUtilities.Turn(turnAngleRight);

    }


}