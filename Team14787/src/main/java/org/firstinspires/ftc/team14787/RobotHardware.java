package org.firstinspires.ftc.team14787;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;
import java.util.ArrayList;

/**
 * Robot hardware configuration class, represents the entire robot
 * and controls movement functionality
 */
public class RobotHardware {

    // Telemetry instance
    private Telemetry telemetry;

    // Robot locational statistics
    private BNO055IMU imu;
    private Orientation lastAngles;
    private double globalAngle, lastAngle, power, correction;
    private boolean touched;

    private PIDController pidRotate;
    private PIDController pidDrive;

    final private double rotateKp = .005;
    final private double rotateKi = 0;
    final private double rotateKd = 0;
    final private double driveKp  = .05;
    final private double driveKi  = 0;
    final private double driveKd  = 0;

    final private MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    final private int GEAR_RATIO = 20;
    final private double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();
    final private double INCHES_PER_REV = 12.3685039;

    // Drive train motors
    private List<DcMotor> driveTrain;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // Hang motor
    DcMotor hang;

    // Arm motors
    DcMotor rotatingArm1;
    DcMotor rotatingArm2;
    DcMotor extendingArm;

    // Vex Servos
    CRServo vex1;
    CRServo vex2;

    // Rev servos
    CRServo rev1;
    CRServo rev2;

    /**
     * Robot hardware constructor including the imu
     */
    RobotHardware(Telemetry telemetry, HardwareMap hardwareMap, BNO055IMU imu) {
        this(telemetry, hardwareMap);
        this.imu = imu;
    }

    /**
     * Robot hardware constructor, configure all motor configurations
     */
    RobotHardware(Telemetry telemetry, HardwareMap hardwareMap) {
        pidRotate = new PIDController(rotateKp, rotateKi, rotateKd);
        pidDrive = new PIDController(driveKp, driveKi, driveKd);

        this.telemetry = telemetry;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        driveTrain = new ArrayList<>();
        driveTrain.add(this.frontLeftDrive);
        driveTrain.add(this.frontRightDrive);
        driveTrain.add(this.backLeftDrive);
        driveTrain.add(this.backRightDrive);

        hang = hardwareMap.get(DcMotor.class, "hang");

        /*
        hang = hardwareMap.get(DcMotor.class, "hang");

        rotatingArm1 = hardwareMap.get(DcMotor.class, "rotatingArm1");
        rotatingArm2 = hardwareMap.get(DcMotor.class, "rotatingArm2");
        extendingArm = hardwareMap.get(DcMotor.class, "extndingArm");

        vex1 = hardwareMap.get(CRServo.class, "vex1");
        vex2 = hardwareMap.get(CRServo.class, "vex2");
        rev1 = hardwareMap.get(CRServo.class, "rev1");
        rev2 = hardwareMap.get(CRServo.class, "rev2");
        */
    }

    /*
    void readSensor() {
        telemetry.addData("Color Sensor Information", "Alpha: %d\nHue: %d\nRGB: (%d, %d, %d)", colorSensor.alpha(), colorSensor.argb(), colorSensor.red(), colorSensor.green(), colorSensor.blue());
    }
    */

    public void turnLeft(double degrees, double speed) {
        lastAngle = getAngle();

        telemetry.addData("Motion", "Last Angle: %.2f\nCurrent Angle: %.2f", lastAngle, getAngle());
        telemetry.update();

        while (getAngle() - lastAngle < degrees) {
            frontLeftDrive.setPower(-speed);
            frontRightDrive.setPower(speed);
            backLeftDrive.setPower(-speed);
            backRightDrive.setPower(speed);

            telemetry.addData("Status", "Turning left");
            telemetry.addData("Motion", "Last Angle: %.2f\nCurrent Angle: %.2f\nTarget Angle: %.2f", lastAngle, getAngle(), degrees + lastAngle);
            telemetry.update();
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
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
        pidRotate.setOutputRange(.20, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0)
            {
                setLeftPower(power);
                setRightPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                setLeftPower(power);
                setRightPower(-power);
            } while (!pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                setLeftPower(power);
                setRightPower(-power);
            } while (!pidRotate.onTarget());

        // turn the motors off.
        setLeftPower(0);
        setRightPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }


    void moveForward(double distance, double power) {
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        correction = pidDrive.performPID(getAngle());

        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();

        double desiredRotations = distance / INCHES_PER_REV;
        double targetEncPosSum = 0;
        double currentEncSum = 0;

        for (DcMotor motor : driveTrain) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            targetEncPosSum += motor.getCurrentPosition() + (7 * GEAR_RATIO * desiredRotations);
        }

        telemetry.addData("Status", "Moving forward %f inches\ntargetEncPosSum = %f", distance, targetEncPosSum);
        telemetry.update();

        int i = 0;

        while (currentEncSum < targetEncPosSum) {
            currentEncSum = 0;
            for (DcMotor motor : driveTrain) {
                motor.setPower(power);
                currentEncSum += motor.getCurrentPosition();
            }
            telemetry.addData("Status", "Iteration: %d\nMoving forward %f inches\ntargetEncPosSum = %f\ncurrentEncSum = %f\nPower = %f", i, distance, targetEncPosSum, currentEncSum, frontRightDrive.getPower());
            telemetry.update();

            i++;
        }

        for (DcMotor motor : driveTrain) {
            motor.setPower(0);
        }
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Set the power of all motors in the drive train
     * @param power Power at which to move motors
     */
    private void setLeftPower(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
    }

    /**
     * Set the power of all motors in the drive train
     * @param power Power at which to move motors
     */
    private void setRightPower(double power) {
        frontRightDrive.setPower(power);
        frontLeftDrive.setPower(power);
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
