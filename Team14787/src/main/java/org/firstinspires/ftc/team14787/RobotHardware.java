package org.firstinspires.ftc.team14787;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
class RobotHardware {
    // Robot locational statistics
    private BNO055IMU imu;
    Orientation lastAngles;
    double globalAngle;

    PIDController pidRotate;
    PIDController pidDrive;

    final private MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    final private double WHEEL_DIAMTER = 3.937;
    final private double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();
    final double INCHES_PER_REV = (WHEEL_DIAMTER * Math.PI) / TICKS_PER_REV;

    // Drive train motors
    List<DcMotor> driveTrain;
    List<DcMotor> frontDrive;
    List<DcMotor> backDrive;
    List<DcMotor> leftDrive;
    List<DcMotor> rightDrive;
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

    // Servos
    CRServo ratchet;
    Servo deployment1;
    Servo deployment2;

    /**
     * Robot hardware constructor including the imu
     *
     * @param hardwareMap Current hardware configuration
     * @param imu Inertial Measurement Unit object, retrieves positional information for the robot
     */
    RobotHardware(HardwareMap hardwareMap, BNO055IMU imu) {
        this(hardwareMap);
        this.imu = imu;

        lastAngles = new Orientation();
    }

    /**
     * Robot hardware constructor, configure all motor configurations
     *
     * @param hardwareMap Current hardware configuration
     */
    RobotHardware(HardwareMap hardwareMap) {
        frontLeftDrive  = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftDrive   = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive  = hardwareMap.dcMotor.get("backRightDrive");

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        /* Motor grouping lists */
        driveTrain = new ArrayList<>();
        driveTrain.add(this.frontLeftDrive);
        driveTrain.add(this.frontRightDrive);
        driveTrain.add(this.backLeftDrive);
        driveTrain.add(this.backRightDrive);

        frontDrive = new ArrayList<>();
        frontDrive.add(this.frontLeftDrive);
        frontDrive.add(this.frontRightDrive);

        backDrive = new ArrayList<>();
        backDrive.add(this.frontLeftDrive);
        backDrive.add(this.frontRightDrive);

        leftDrive = new ArrayList<>();
        leftDrive.add(this.frontLeftDrive);
        leftDrive.add(this.frontRightDrive);

        rightDrive = new ArrayList<>();
        rightDrive.add(this.frontLeftDrive);
        rightDrive.add(this.frontRightDrive);

        hang = hardwareMap.dcMotor.get("hang");

        ratchet = hardwareMap.crservo.get("ratchet");
        //deployment1 = hardwareMap.servo.get("deployment1");
        //deployment2 = hardwareMap.servo.get("deployment2");

        /*
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

    /**
     * Updates the global angle using the imu
     * @return The measured angle in degrees
     */
    double getAngle() {
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
     * Reset drive train encoder positions
     */
    void resetEncoders() {
        for (DcMotor motor : driveTrain) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Calculate the sum of the left motor positions
     * @return Sum of the left encoder positons
     */
    double getLeftTicks() {
        return -(frontLeftDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition());
    }

    /**
     * Calculate the sum of the right motor positions
     * @return Sum of the right encoder positons
     */
    double getRightTicks() {
        return -(frontLeftDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition());
    }

    /**
     * Calculate the sum of the front motor positions
     * @return Sum of the right encoder positons
     */
    double getFrontTicks() {
        return -(frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition());
    }

    /**
     * Calculate the sum of the front motor positions
     * @return Sum of the right encoder positons
     */
    double getBackTicks() {
        return -(backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition());
    }

    /**
     * Calculate the desired tick rotation of the left motors
     * @param distance The distance to travel
     * @return Sum of the projected left motor encoder positions
     */
    double calculateLeftTicks(double distance) {
        return -frontLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV) + -backLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV);
    }

    /**
     * Calculate the desired tick rotation of the right motors
     * @param distance The distance to travel
     * @return Sum of the projected right motor encoder positions
     */
    double calculateRightTicks(double distance) {
        return frontLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV) + -backLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV);
    }

    /**
     * Calculate the desired tick rotation of the front motors
     * @param distance The distance to travel
     * @return Sum of the projected front motor encoder positions
     */
    double calculateFrontTicks(double distance) {
        return -frontLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV) + -backLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV);
    }

    /**
     * Calculate the desired tick rotation of the back motors
     * @param distance The distance to travel
     * @return Sum of the projected back motor encoder positions
     */
    double calculateBackTicks(double distance) {
        return frontLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV) + -backLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV);
    }

    /**
     * Change motors to or from left strafe orientation
     */
    void toggleLeftStrafe() {
        if (frontLeftDrive.getDirection().equals(DcMotor.Direction.FORWARD)) {
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        } else {
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        if (backRightDrive.getDirection().equals(DcMotor.Direction.FORWARD)) {
            backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        } else {
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    /**
     * Change motors to or from right strafe orientation
     */
    void toggleRightStrafe() {
        if (frontRightDrive.getDirection().equals(DcMotor.Direction.FORWARD)) {
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        } else {
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        if (backLeftDrive.getDirection().equals(DcMotor.Direction.FORWARD)) {
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        } else {
            backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Set the power for each motor in the drive train
     * @param power Power at which to move motors
     */
    void setDrivePower(double power) {
        for (DcMotor motor : driveTrain) {
            motor.setPower(power);
        }
    }

    /**
     * Set the power of all motors in the drive train
     * @param power Power at which to move motors
     */
    void setLeftPower(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
    }

    /**
     * Set the power of all motors in the drive train
     * @param power Power at which to move motors
     */
    void setRightPower(double power) {
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    /**
     * Set the power of all motors in the drive train
     * @param power Power at which to move motors
     */
    void setFrontPower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
    }

    /**
     * Set the power of all motors in the drive train
     * @param power Power at which to move motors
     */
    void setBackPower(double power) {
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }
}
