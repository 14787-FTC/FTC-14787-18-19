package org.firstinspires.ftc.team14787;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="AutonOpMode", group="Autonomous")
public class AutonOpMode extends LinearOpMode {

    /** TFOD configuration and label */
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /** Vuforia API Key */
    private static final String VUFORIA_KEY = "AW3RM/7/////AAABmdYUnHQNFUBUtLcj9yNEiU1eT9NilwrFUzWl2FV1fFXafePbmAt1mX9m1x5ZSvHHFXCHKPWLGD2w/X14S4Zie69lPlJzVvT1JE+SJCgDiNabghLZdKai9ITjNLnRliOfaGcGF/sEgr7AP/oZkc4nWetITL3wve+hclRlqcvEJRvixMgBrCluh8F0L58pKEZT6MUVtXes4lEx5agdsWOvgTkLYzxqaSHx8f+sO/qXgAojxRzEdcJ5o2MdgpN9YuZJoSSSvpT1yvPpp5OuVhdH5CpkNNnP0qVHdnvH5QK7g5TC2bKeTvx60DRroVa5/U4ZrMyQqsRnz78gagsFsXqKa+xf1HS324Y5zoUZ/a2UFlfx";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    /** Robot representation */
    private RobotHardware robot;

    /** Tunable speeds */
    private static final double DRIVE_POWER = 0.05;
    private static final double STRAFE_POWER = 0.1;
    private static final double ROTATE_MIN_POWER = 0.25;
    private static final double ROTATE_MAX_POWER = 0.4;

    /** PID Constants */
    final private double driveKp  = .05;
    final private double driveKi  = 0;
    final private double driveKd  = 0;
    final private double rotateKp = .005;
    final private double rotateKi = 0;
    final private double rotateKd = 0;

    private double correction;

    /** PID Controlles for drive and rotation */
    private final PIDController pidDrive = new PIDController(driveKp, driveKi, driveKd);
    private final PIDController pidRotate = new PIDController(rotateKp, rotateKi, rotateKd);

    /** Mineral location enumerable */
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
            telemetry.addData("Status", "This device is not compatible with TFOD");
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

        // Calibrate IMU
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("IMU Calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Instantiate robot subsystem
        robot = new RobotHardware(hardwareMap, imu);

        // Sampling with TFOD
        MineralLocation goldLocation = null;

        // Activate Tensor Flow Object Detection.
        if (tfod != null) {
            tfod.activate();
        }

        // Detect relative gold mineral position via recognizing the two far left minerals
        while (goldLocation == null) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("Status", "# Object Detected: %d", updatedRecognitions.size());
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

        // Deactive TFOD
        if (tfod != null) tfod.deactivate();

        telemetry.addData("Status", "Gold location found, waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Mode", "Knocking off gold piece");
        telemetry.update();

        //moveToEncoderPosition(robot.hang, 1251);

        // Detach from hang
        robot.hang.setPower(-.5);
        robot.ratchet.setPower(1);
        sleep(500);
        robot.ratchet.setPower(0);
        robot.hang.setPower(1);

        // Fork behavior based off of detected gold position, eventually returns to center
        switch (goldLocation) {
            case LEFT: leftDetected(); break;
            case CENTER: centerDetected(); break;
            case RIGHT: rightDetected(); break;
        }

    }

    /**
     * Movement methods based on goldLocation
     */
    private void leftDetected() {
        moveForward(12, DRIVE_POWER);
        strafeLeft(20, STRAFE_POWER);
        moveForward(12, DRIVE_POWER);
        moveBackward(12, DRIVE_POWER);
        rotate(90, ROTATE_MIN_POWER, ROTATE_MIN_POWER);
    }

    private void centerDetected() {
        moveForward(24, DRIVE_POWER);
        moveBackward(12, DRIVE_POWER);
    }

    private void rightDetected() {
        moveForward(12, DRIVE_POWER);
        strafeRight(20, STRAFE_POWER);
        moveForward(12, DRIVE_POWER);
        moveBackward(12, DRIVE_POWER);
        rotate(90, ROTATE_MIN_POWER, ROTATE_MIN_POWER);
    }

    /* Basic movement API */

    /**
     * Rotate a motor to a desired encoder position
     * @param motor Motor to turn
     * @param power Power at which to turn motor
     * @param pos Desired ticks
     */
    private void moveToEncoderPosition(DcMotor motor, double power, int pos) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (motor.getCurrentPosition() > pos) {
            telemetry.addData("Moving down", "Target Position: %d\nCurrent Position: %d", pos, motor.getCurrentPosition());
            telemetry.update();
            motor.setPower(-1);
            while (motor.getCurrentPosition() > pos && opModeIsActive()) {
                sleep(10);
            }
        } else {
            telemetry.addData("Moving up", "Target Position: %d\nCurrent Position: %d", pos, motor.getCurrentPosition());
            telemetry.update();
            motor.setPower(1);
            while (motor.getCurrentPosition() < pos && opModeIsActive()) {
                sleep(10);
            }
        }
        motor.setPower(0);
    }

    /**
     * Rotate the robot using PID and the expansion hub's internal IMU
     * @param degrees How far to turn
     * @param minPower Minimum power to rotate motors for adjustments
     * @param maxPower Maximum power at which to run motors
     */
    private void rotate(int degrees, double minPower, double maxPower) {
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
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(minPower, maxPower);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.getAngle() == 0)
            {
                robot.setLeftPower(-maxPower);
                robot.setRightPower(maxPower);
                sleep(100);
            }

            do
            {
                maxPower = pidRotate.performPID(robot.getAngle()); // power will be - on right turn.
                robot.setLeftPower(maxPower);
                robot.setRightPower(-maxPower);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else {
            do {
                maxPower = pidRotate.performPID(robot.getAngle()); // power will be + on left turn.
                robot.setLeftPower(maxPower);
                robot.setRightPower(-maxPower);
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
     * Strafe left with a similar mechanism to forward movement
     * @param distance Distance, in inches, to strafe
     * @param power Power at which to move motors
     */
    private void strafeLeft(double distance, double power) {
        distance = distance + 1;
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        robot.resetEncoders();
        robot.toggleLeftStrafe();

        double frontEncSum = robot.calculateFrontTicks(distance);
        double backEncSum  = robot.calculateBackTicks(distance);

        telemetry.addData("Status", "Moving forward %f inches\nDesired front encoder position sum = %f\nDesired left encoder position sum = %f", distance, frontEncSum, backEncSum);
        telemetry.update();

        while (robot.getFrontTicks() < frontEncSum && robot.getBackTicks() < backEncSum && opModeIsActive()) {
            correction = pidDrive.performPID(robot.getAngle());

            telemetry.addData("1 IMU Heading", robot.lastAngles.firstAngle);
            telemetry.addData("2 Global Heading", robot.globalAngle);
            telemetry.addData("3 Correction", correction);

            robot.setLeftPower(-power + correction);
            robot.setRightPower(-power);

            telemetry.addData("Moved forward %f inches", robot.getLeftTicks() * robot.INCHES_PER_REV);
            telemetry.addData("Status", "leftEncSum: %f, frontEncSum: %f\nbackEncSum: %f, rightTicks: %f", frontEncSum, backEncSum, robot.getLeftTicks(), robot.getRightTicks());
            telemetry.update();
        }

        robot.toggleLeftStrafe();
        robot.setDrivePower(0);
    }

    /**
     * Strafe right with a similar mechanism to forward movement
     * @param distance Distance, in inches, to strafe
     * @param power Power at which to move motors
     */
    private void strafeRight(double distance, double power) {
        distance = distance + 1;
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        robot.resetEncoders();
        robot.toggleRightStrafe();

        double frontEncSum = robot.calculateFrontTicks(distance);
        double backEncSum  = robot.calculateBackTicks(distance);

        telemetry.addData("Status", "Moving forward %f inches\nDesired front encoder position sum = %f\nDesired left encoder position sum = %f", distance, frontEncSum, backEncSum);
        telemetry.update();

        while (robot.getFrontTicks() < frontEncSum && robot.getBackTicks() < backEncSum && opModeIsActive()) {
            correction = pidDrive.performPID(robot.getAngle());

            telemetry.addData("1 IMU Heading", robot.lastAngles.firstAngle);
            telemetry.addData("2 Global Heading", robot.globalAngle);
            telemetry.addData("3 Correction", correction);

            robot.setLeftPower(-power);
            robot.setRightPower(-power + correction);

            telemetry.addData("Moved forward %f inches", robot.getLeftTicks() * robot.INCHES_PER_REV);
            telemetry.addData("Status", "leftEncSum: %f, frontEncSum: %f\nbackEncSum: %f, rightTicks: %f", frontEncSum, backEncSum, robot.getLeftTicks(), robot.getRightTicks());
            telemetry.update();
        }

        robot.toggleRightStrafe();
        robot.setDrivePower(0);
    }

    /**
     * Move the robot forward a amount, treating the drive as a tank train
     * @param distance Distance, in inches, to move forward
     * @param power Power at which to move motors
     */
    private void moveForward(double distance, double power) {
        distance = distance - 1;

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        robot.resetEncoders();

        double leftEncSum  = robot.calculateLeftTicks(distance);
        double rightEncSum = robot.calculateRightTicks(distance);

        telemetry.addData("Status", "Moving forward %f inches\nDesired left encoder position sum = %f\nDesired right encoder position sum = %f", distance, leftEncSum, rightEncSum);
        telemetry.update();

        while (robot.getLeftTicks() < leftEncSum && robot.getRightTicks() < rightEncSum && opModeIsActive()) {
            correction = pidDrive.performPID(robot.getAngle());

            telemetry.addData("1 IMU Heading", robot.lastAngles.firstAngle);
            telemetry.addData("2 Global Heading", robot.globalAngle);
            telemetry.addData("3 Correction", correction);

            robot.setLeftPower(-power + correction);
            robot.setRightPower(-power);

            telemetry.addData("Moved forward %f inches", robot.getLeftTicks() * robot.INCHES_PER_REV);
            telemetry.addData("Status" , "leftEncSum: %f, rightEncSum: %f\nleftTicks: %f, rightTicks: %f", leftEncSum, rightEncSum, robot.getLeftTicks(), robot.getRightTicks());
            telemetry.update();
        }

        robot.setDrivePower(0);
    }

    /**
     * Move the robot forward a amount, treating the drive as a tank train
     * @param distance Distance, in inches, to move forward
     * @param power Power at which to move motors
     */
    private void moveBackward(double distance, double power) {
        reverseDrive();
        moveForward(distance, power);
        reverseDrive();
    }

    /**
     * Reverse the direction of all motors in the drive train
     */
    private void reverseDrive() {
        for (DcMotor motor : robot.driveTrain) {
            toggleMotorDirection(motor);
        }
    }

    /**
     * Toggles the direction of a passed motor
     * @param motor Motor to reverse
     */
    private void toggleMotorDirection(DcMotor motor) {
        if (motor.getDirection().equals(DcMotor.Direction.FORWARD)) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
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
