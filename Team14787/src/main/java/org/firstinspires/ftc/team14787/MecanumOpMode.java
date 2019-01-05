package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.Collections;

/**
 * Default TeleOp Move for mecanum drive
 */
@TeleOp(name="Mecanum Drive", group="Linear Opmode")
public class MecanumOpMode extends OpMode {

    /** Robot representation */
    private RobotHardware robot;

    /** Default drive power */
    private final double DRIVE_POWER = 0.5;

    /** Total runtime to output for debugging */
    private ElapsedTime runtime;

    /** Controller inputs */
    private boolean dPadUp;
    private boolean dPadDown;

    /**
     * Initialize robot
     */
    @Override
    public void init() {
        runtime = new ElapsedTime();

        // Instantiate robot with hardware specifications
        robot = new RobotHardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Reset team marker deployment servos
        robot.deployment1.setPosition(0);
        robot.deployment2.setPosition(0);
    }

    /**
     * Main control loop, handle controller input and motor speeds
     */
    @Override
    public void loop() {
        // Gamepad configuration and calculation variables
        double r, robotAngle, rightX, p1, p2, p3, p4;

        // Calculate required power for each wheel based off of controller input and robot position
        r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        rightX = gamepad1.right_stick_x;
        dPadUp = gamepad1.dpad_up;
        dPadDown = gamepad1.dpad_down;

        p1 = r * Math.cos(robotAngle) - rightX;
        p2 = r * Math.sin(robotAngle) + rightX;
        p3 = r * Math.sin(robotAngle) - rightX;
        p4 = r * Math.cos(robotAngle) + rightX;

        // Set each respective power
        robot.frontLeftDrive.setPower(p1);
        robot.frontRightDrive.setPower(p2);
        robot.backLeftDrive.setPower(p3);
        robot.backRightDrive.setPower(p4);

        // Manage hang control
        if (dPadUp) {
            robot.hang.setPower(1);
        } else if (dPadDown) {
            robot.hang.setPower(-1);
        } else {
            robot.hang.setPower(0);
        }

        // Provide debugging information
        telemetry.addData("Status", "Runtime: %s", runtime.toString());
        telemetry.addData("Drive Train", "Front Left: %.2f\nFront Right: %.2f\nBack Left: %.2f\nBack Rigth: %.2f", p1, p2, p3, p4);
        telemetry.update();
    }
}
