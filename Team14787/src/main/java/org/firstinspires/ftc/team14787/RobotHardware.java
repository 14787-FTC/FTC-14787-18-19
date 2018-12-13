package org.firstinspires.ftc.team14787;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
    private double lastAngle, power, correction;
    private boolean touched;

    private final double drivePidKp = 1;     // Tuning variable for PID.
    private final double drivePidTi = 1.0;   // Eliminate integral error in 1 sec.
    private final double drivePidTd = 0.1;   // Account for error in 0.1 sec.
    // Protect against integral windup by limiting integral term.
    private final double drivePidIntMax = 1;  // Limit to max speed.
    private final double driveOutMax = 1.0;  // Motor output limited to 100%.

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

    // Color sensor
    private ColorSensor colorSensor;

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

        //colorSensor = hardwareMap.colorSensor.get("color");

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

    void readSensor() {
        telemetry.addData("Color Sensor Information", "Alpha: %d\nHue: %d\nRGB: (%d, %d, %d)", colorSensor.alpha(), colorSensor.argb(), colorSensor.red(), colorSensor.green(), colorSensor.blue());
    }

    public void turnLeft(double degrees, double speed) {
        lastAngle = getGlobalAngle();

        telemetry.addData("Motion", "Last Angle: %.2f\nCurrent Angle: %.2f", lastAngle, getGlobalAngle());
        telemetry.update();

        while (getGlobalAngle() - lastAngle < degrees) {
            frontLeftDrive.setPower(-speed);
            frontRightDrive.setPower(speed);
            backLeftDrive.setPower(-speed);
            backRightDrive.setPower(speed);

            telemetry.addData("Status", "Turning left");
            telemetry.addData("Motion", "Last Angle: %.2f\nCurrent Angle: %.2f\nTarget Angle: %.2f", lastAngle, getGlobalAngle(), degrees + lastAngle);
            telemetry.update();
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void turnRight(double degrees, double speed) {
        lastAngle = getGlobalAngle();

        telemetry.addData("Motion", "Last Angle: %.2f\nCurrent Angle: %.2f", lastAngle, getGlobalAngle());
        telemetry.update();

        while (lastAngle - getGlobalAngle() < degrees) {
            frontLeftDrive.setPower(speed);
            frontRightDrive.setPower(-speed);
            backLeftDrive.setPower(speed);
            backRightDrive.setPower(-speed);

            telemetry.addData("Status", "Turning left");
            telemetry.addData("Motion", "Last Angle: %.2f\nCurrent Angle: %.2f\nTarget Angle: %.2f", lastAngle, getGlobalAngle(), degrees + lastAngle);
            telemetry.update();
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    void moveForward(double distance, double speed) {
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

        while (currentEncSum < targetEncPosSum) {
            currentEncSum = 0;
            for (DcMotor motor : driveTrain) {
                motor.setPower(speed);
                currentEncSum += motor.getCurrentPosition();
            }
            telemetry.addData("Status", "Moving forward %f inches\ntargetEncPosSum = %f\ncurrentEncSum = %f\nPower = %f", distance, targetEncPosSum, currentEncSum, frontRightDrive.getPower());
            telemetry.update();
        }

        for (DcMotor motor : driveTrain) {
            motor.setPower(0);
        }
    }

    private double getGlobalAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
