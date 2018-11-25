package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "gold mineral tracking")
public class GoldMineralTracking extends LinearOpMode {
    DcMotor[] drivetrainDC = new DcMotor[4];
    //drivetrainDC[0] = RIGHT
    //drivetrainDC[1] = LEFT
    boolean findCube = false;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public int cubePlace = 0;// 0 = NOT HERE, 1 = RIGHT (in camera), 2 = LEFT (in camera)
    public boolean noMotor = true;
    int goldIndex = -1;
    boolean foundGold = false;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = " ATgDONj/////AAABmW0G/nQirUMiumnzPc6Pl8oJhBOCC2qoUq0BWhir9YWcBFDlhZUfSwATcQArcyyLxIOV21sHaYJeeQEJZfIJ+4spBn3oJ/DfycsbPaNs87+TRpM46/vbUkj1Ok+NtZ/eqMhmMXjFC8dgdCfbCt0aMxoBNzDw4+v28abG+hjUCjVYf86Jq1m7R942XCjw0yhOZqTXWIp3WAZDXY/PdWGQGY/zWae0l6TAZ6Z27t1xYJdkkpLqEsbKM3ZprvtgIs8AsWS9Tri2892OHq2CnCL+1ZHHXKPdxON3fiC1Gd3oihwPhTUReNw0VAg9yeVsVa1UQg7ea9K6WpmVto0FG+T2/LV8uq/3Mp/NHWiNizw2DM4h";
    ;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override

    public void runOpMode() {
            initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (!noMotor) {
            drivetrainDC[0] = hardwareMap.get(DcMotor.class, "rightFront");
            drivetrainDC[1] = hardwareMap.get(DcMotor.class, "rightBack");
            drivetrainDC[2] = hardwareMap.get(DcMotor.class, "leftBack");
            drivetrainDC[3] = hardwareMap.get(DcMotor.class, "leftFront");
        }
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        waitForStart();
        telemetry.addLine("follow cube 0:");
        telemetry.update();

        if (tfod != null) {
            tfod.activate();
        }
        followCube(0.2);
//        if (getCube() == 0) {
//
//        } else if (getCube() == 1) {
//
//        } else if (getCube() == 2) {
//
//        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }


    private void followCube(double power) {
        telemetry.addLine("follow cube 1:");
        telemetry.update();

        double distanceFromRight = 0;
        double distanceFromLeft = 0;
        double middleCubeX = 0;
        double k = 0.0007; //EDEN
        double[] addToMotors = new double[2];


        List<Recognition> updatedRecognitions = null;


//        while (opModeIsActive()) {//TODO: ADD CONDITION
//            telemetry.addLine("search cube 1");
//            telemetry.update();
//            if (tfod != null)
//                updatedRecognitions = tfod.getUpdatedRecognitions();
//
//            if (updatedRecognitions != null
//               &&!updatedRecognitions.isEmpty()
//                        && updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL))
//                    break;
//        }


        while (opModeIsActive()) {
            if (tfod != null) {
                updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null
                        && !updatedRecognitions.isEmpty()) {
                    telemetry.addLine("Object found");
                    telemetry.update();
                    for (int i = 0; i < updatedRecognitions.size(); i++)
                        if (updatedRecognitions.get(i).getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldIndex = i;
                            telemetry.addData("Found gold",i);
                            telemetry.update();
                            sleep(200);
                            break;
                        } else
                            goldIndex = -1;
                    if (goldIndex != -1) {
                        foundGold = true;
                        middleCubeX = ((updatedRecognitions.get(goldIndex).getLeft() + updatedRecognitions.get(goldIndex).getRight()) / 2);
                        distanceFromRight = 700 - middleCubeX;
                        distanceFromLeft = middleCubeX;
                        telemetry.addLine("follow cube 4:");
                        addToMotors[0] = k * distanceFromRight;  //RIGHT
                        addToMotors[1] = k * distanceFromLeft; //LEFT
                        telemetry.addData("distance From Right:", addToMotors[0]);
                        telemetry.addData("distance From Left:", addToMotors[1]);
                        telemetry.update();

                        if (!noMotor) {
                            drivetrainDC[0].setPower(addToMotors[0] + power);//RIGHT Front
                            drivetrainDC[1].setPower(addToMotors[0] + power);//Right Back
                            drivetrainDC[2].setPower(addToMotors[1] + power);//Left Back
                            drivetrainDC[3].setPower(addToMotors[1] + power);//LEFT Front
                        }
                    }
                } else if (foundGold) {
                    telemetry.addLine("dont see cube");
                    telemetry.update();
                    // break;
                } else {
                    foundGold = false;
                    telemetry.addLine("Nothing detected");
                    telemetry.update();
                }
            }


//            powerAdd ToMotors[0] = getPowerMotor()[0];//RIGHT
//            powerAddToMotors[1] = getPowerMotor()[1];//LEFT


            //   break;

        }

    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}