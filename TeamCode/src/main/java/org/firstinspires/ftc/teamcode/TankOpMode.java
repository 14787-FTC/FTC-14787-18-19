package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Tank OpMode", group="Linear Opmode")
public class TankOpMode extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() {
        // Set initial power for each tread row
        double leftPower = 0;
        double rightPower = 0;

        // Create and configure motor instances
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Calculate power for tank configuration
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

            // Set motor's speed based on retrieved values
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
