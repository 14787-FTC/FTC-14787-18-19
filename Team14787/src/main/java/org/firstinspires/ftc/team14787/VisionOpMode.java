package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Vision Testing", group="Autonomous")
public class VisionOpMode extends LinearOpMode {

    private RobotHardware robot;

    /** TFOD configuration and label */
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /** Vuforia API Key */
    private static final String VUFORIA_KEY = "AW3RM/7/////AAABmdYUnHQNFUBUtLcj9yNEiU1eT9NilwrFUzWl2FV1fFXafePbmAt1mX9m1x5ZSvHHFXCHKPWLGD2w/X14S4Zie69lPlJzVvT1JE+SJCgDiNabghLZdKai9ITjNLnRliOfaGcGF/sEgr7AP/oZkc4nWetITL3wve+hclRlqcvEJRvixMgBrCluh8F0L58pKEZT6MUVtXes4lEx5agdsWOvgTkLYzxqaSHx8f+sO/qXgAojxRzEdcJ5o2MdgpN9YuZJoSSSvpT1yvPpp5OuVhdH5CpkNNnP0qVHdnvH5QK7g5TC2bKeTvx60DRroVa5/U4ZrMyQqsRnz78gagsFsXqKa+xf1HS324Y5zoUZ/a2UFlfx";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    /** Mineral location enumerable */
    enum MineralLocation {
        LEFT, CENTER, RIGHT
    }

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        //robot.hang.setPower(-1);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Status", "This device is not compatible with TFOD");
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Activate Tensor Flow Object Detection.
        if (tfod != null) {
            tfod.activate();
        }

        MineralLocation goldLocation;

        // Detect relative gold mineral position via recognizing the two far left minerals
        while (opModeIsActive()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    int goldCount = 0, silverCount = 0;
                    for (Recognition r : updatedRecognitions) {
                        if (r.getLabel().equals(LABEL_GOLD_MINERAL)) goldCount++;
                        else silverCount++;
                    }
                    telemetry.addData("Detected Objects", "Total: %d\nGold: %d, Silver: %d", updatedRecognitions.size(), goldCount,silverCount);
                    if (updatedRecognitions.size() == 2) {
                        Recognition mineral1 = updatedRecognitions.get(0);
                        Recognition mineral2 = updatedRecognitions.get(1);

                        if (mineral1.getLabel().equals(LABEL_GOLD_MINERAL) || mineral2.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            if (mineral1.getLabel().equals(LABEL_GOLD_MINERAL) && mineral1.getRight() < mineral2.getRight()) {
                                goldLocation = MineralLocation.CENTER;
                            } else {
                                goldLocation = MineralLocation.LEFT;
                            }
                        } else {
                            goldLocation = MineralLocation.RIGHT;
                        }

                        telemetry.addData("Gold Location", goldLocation.toString());
                    }
                    telemetry.update();
                }
            }
        }

        // Deactive TFOD
        if (tfod != null) tfod.deactivate();
    }

    /**
     * Initialize the Vuforia localization engine
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine
    }

    /**
     * Initialize the Tensor Flow Object Detection engine
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
