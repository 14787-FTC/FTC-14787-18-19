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
@TeleOp(name="Tank Drive", group="Linear Opmode")
public class TankOpMode extends OpMode {

    /** Robot representation */
    private RobotHardware robot;

    /** Total runtime to output for debugging */
    private ElapsedTime runtime;

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
        robot.setLeftPower(gamepad1.left_stick_y);
        robot.setRightPower(gamepad1.right_stick_y);

        // Provide debugging information
        telemetry.addData("Status", "Runtime: %s", runtime.toString());
        telemetry.update();
    }
}
