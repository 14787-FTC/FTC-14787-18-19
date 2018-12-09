package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

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

    // Color sensor
    ColorSensor colorSensor;

    /**
     * Robot hardware constructor including the imu
     */
    public RobotHardware(Telemetry telemetry, HardwareMap hardwareMap, BNO055IMU imu) {
        this(telemetry, hardwareMap);
        this.imu = imu;
    }

    /**
     * Robot hardware constructor, configure all motor configurations
     */
    public RobotHardware(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        driveTrain = new ArrayList<DcMotor>();
        driveTrain.add(this.frontLeftDrive);
        driveTrain.add(this.frontRightDrive);
        driveTrain.add(this.backLeftDrive);
        driveTrain.add(this.backRightDrive);

        colorSensor = hardwareMap.colorSensor.get("color");

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

    public void readSensor() {
        telemetry.addData("Color Sensor Information", "Alpha: %d\nHue: %d\nRGB: (%d, %d, %d)", colorSensor.alpha(), colorSensor.argb(), colorSensor.red(), colorSensor.green(), colorSensor.blue());
    }

    public void left(double degrees, double speed) {
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

    public void right(double degrees, double speed) {
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

    public double getGlobalAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
