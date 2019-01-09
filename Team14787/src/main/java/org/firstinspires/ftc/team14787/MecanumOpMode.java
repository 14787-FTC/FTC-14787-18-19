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
    }

    /**
     * Main control loop, handle controller input and motor speeds
     */
    @Override
    public void loop() {
        dPadUp = gamepad1.dpad_up;
        dPadDown = gamepad1.dpad_down;

        // Convert joysticks to desired motion.
        Mecanum.Motion motion = Mecanum.joystickToMotion(
                gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, gamepad1.right_stick_y);


        // Convert desired motion to wheel powers, with power clamping.
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        robot.frontLeftDrive.setPower(-wheels.frontLeft);
        robot.frontRightDrive.setPower(-wheels.frontRight);
        robot.backLeftDrive.setPower(-wheels.backLeft);
        robot.backRightDrive.setPower(-wheels.backRight);

        // Manage hang control
        if (dPadUp) {
            robot.hang.setPower(1);
        } else if (dPadDown) {
            robot.hang.setPower(-1);
        } else {
            robot.hang.setPower(0);
        }

        /*
        // Gamepad configuration and calculation variables
        double r, robotAngle, rightX, p1, p2, p3, p4;

        // Calculate required power for each wheel based off of controller input and robot position
        r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        rightX = gamepad1.right_stick_x;
        p1 = r * Math.cos(robotAngle) - rightX;
        p2 = r * Math.sin(robotAngle) + rightX;
        p3 = r * Math.sin(robotAngle) - rightX;
        p4 = r * Math.cos(robotAngle) + rightX;

        robot.frontLeftDrive.setPower(p1);
        robot.frontRightDrive.setPower(p2);
        robot.backLeftDrive.setPower(p3);
        robot.backRightDrive.setPower(p4);

        */

        // Provide debugging information
        telemetry.addData("Status", "Runtime: %s", runtime.toString());
        telemetry.addData("Drive Train", "Front Left: %.2f\nFront Right: %.2f\nBack Left: %.2f\nBack Rigth: %.2f", wheels.frontLeft, wheels.frontRight, wheels.backLeft, wheels.backRight);
        telemetry.update();
    }
}
