package org.firstinspires.ftc.team14787;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    private RobotHardware robot;

    final private double rotateKp = .005;
    final private double rotateKi = 0;
    final private double rotateKd = 0;
    final private double driveKp  = .05;
    final private double driveKi  = 0;
    final private double driveKd  = 0;

    private double correction;

    private final PIDController pidRotate = new PIDController(rotateKp, rotateKi, rotateKd);
    private final PIDController pidDrive = new PIDController(driveKp, driveKi, driveKd);

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

        telemetry.addData("IMU Calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();

        robot = new RobotHardware(hardwareMap, imu);

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

        moveForward(6, .25);

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

    /* Basic movement API */

    /**
     * Rotate the robot using PID and the expansion hub's internal IMU
     * @param degrees How far to turn
     * @param power Power at which to move motors
     */
    private void rotate(int degrees, double power) {
        // restart imu angle tracking.
        robot.resetAngle();

        // Start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.3, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.getAngle() == 0)
            {
                robot.setLeftPower(-power);
                robot.setRightPower(power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(robot.getAngle()); // power will be - on right turn.
                robot.setLeftPower(power);
                robot.setRightPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else {
            do {
                power = pidRotate.performPID(robot.getAngle()); // power will be + on left turn.
                robot.setLeftPower(power);
                robot.setRightPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }

        // turn the motors off.
        robot.setLeftPower(0);
        robot.setRightPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        robot.resetAngle();
    }

    /**
     * Move the robot forward a amount, treating the drive as a tank train
     * @param distance Distance, in inches, to move forward
     * @param power Power at which to move motors
     */
    private void moveForward(double distance, double power) {
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        double desiredRotations = distance / robot.INCHES_PER_REV;
        double targetEncPosSum = 0;
        double currentEncSum = 0;

        for (DcMotor motor : robot.driveTrain) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            targetEncPosSum += motor.getCurrentPosition() + (desiredRotations * robot.TICKS_PER_REV);
        }

        telemetry.addData("Status", "Moving forward %f inches\nDesired rotations per motor = %f", distance, desiredRotations);
        telemetry.update();

        int i = 0;

        while (currentEncSum < targetEncPosSum && opModeIsActive()) {
            correction = pidDrive.performPID(robot.getAngle());

            telemetry.addData("1 imu heading", robot.lastAngles.firstAngle);
            telemetry.addData("2 global heading", robot.globalAngle);
            telemetry.addData("3 correction", correction);

            currentEncSum = 0;
            robot.setLeftPower(-power + correction);
            robot.setRightPower(-power);

            for (DcMotor motor : robot.driveTrain) {
                currentEncSum -= motor.getCurrentPosition();
            }

            telemetry.addData("Status", "Iteration: %d\nMoving forward %f inches\ntargetEncPosSum = %f\ncurrentEncSum = %f\nPower = %f", i, distance, targetEncPosSum, currentEncSum, robot.frontLeftDrive.getPower());
            telemetry.update();

            i++;
        }

        robot.setDrivePower(0);
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
