package org.firstinspires.ftc.team14787;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="AutonOpMode", group="Autonomous")
public class AutonOpMode extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AW3RM/7/////AAABmdYUnHQNFUBUtLcj9yNEiU1eT9NilwrFUzWl2FV1fFXafePbmAt1mX9m1x5ZSvHHFXCHKPWLGD2w/X14S4Zie69lPlJzVvT1JE+SJCgDiNabghLZdKai9ITjNLnRliOfaGcGF/sEgr7AP/oZkc4nWetITL3wve+hclRlqcvEJRvixMgBrCluh8F0L58pKEZT6MUVtXes4lEx5agdsWOvgTkLYzxqaSHx8f+sO/qXgAojxRzEdcJ5o2MdgpN9YuZJoSSSvpT1yvPpp5OuVhdH5CpkNNnP0qVHdnvH5QK7g5TC2bKeTvx60DRroVa5/U4ZrMyQqsRnz78gagsFsXqKa+xf1HS324Y5zoUZ/a2UFlfx";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    enum MineralLocation {
        LEFT, CENTER, RIGHT
    }

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Configure IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode           = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "Calibrating IMU");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Waiting for start");
        telemetry.addData("IMU Calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();

        RobotHardware robot = new RobotHardware(telemetry, hardwareMap, imu);

        MineralLocation goldLocation = null;

        /* Activate Tensor Flow Object Detection.
        if (tfod != null) {
            tfod.activate();
        }*/

        /*
        while (goldLocation == null) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 2) {
                        Recognition mineral1 = updatedRecognitions.get(0);
                        Recognition mineral2 = updatedRecognitions.get(1);

                        if (mineral1.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            if (mineral1.getLeft() < mineral2.getLeft()) {
                                goldLocation = MineralLocation.LEFT;
                            } else {
                                goldLocation = MineralLocation.CENTER;
                            }
                        } else if (mineral2.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            if (mineral2.getLeft() < mineral1.getLeft()) {
                                goldLocation = MineralLocation.LEFT;
                            } else {
                                goldLocation = MineralLocation.CENTER;
                            }
                        } else {
                            goldLocation = MineralLocation.RIGHT;
                        }
                    }
                    telemetry.update();
                }
            }
        }


        if (tfod != null) {
            tfod.deactivate();
        }
        */

        telemetry.addData("Status", "Gold location found, waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Move forward for 250 milliseconds
        /*
        for (DcMotor motor : robot.driveTrain) {
            motor.setPower(1);
        }
        sleep(250);
        */

        telemetry.addData("Mode", "Knocking off gold piece");
        telemetry.update();

        //robot.test.setPower(1);
        //sleep(500);
        //robot.test.setPower(0);

        robot.rotate(90, .5);

        /*
        switch (goldLocation) {
            case LEFT: leftFork(); break;
           // case CENTER: centerFork(); break;
            //case RIGHT: rigthFork(); break;
        }
        */
    }

    /**
     * Movement methods based on goldLocation
     */
    private void leftFork() {

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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
