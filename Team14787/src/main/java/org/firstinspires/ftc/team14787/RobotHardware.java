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
    // Robot locational statistics
    private BNO055IMU imu;
    Orientation lastAngles;
    double globalAngle;

    PIDController pidRotate;
    PIDController pidDrive;

    final private MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();
    final double INCHES_PER_REV = 12.3685039;

    // Drive train motors
    List<DcMotor> driveTrain;
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
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        driveTrain = new ArrayList<>();
        driveTrain.add(this.frontLeftDrive);
        driveTrain.add(this.frontRightDrive);
        driveTrain.add(this.backLeftDrive);
        driveTrain.add(this.backRightDrive);

        hang = hardwareMap.get(DcMotor.class, "hang");

        hang = hardwareMap.get(DcMotor.class, "hang");

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
}
