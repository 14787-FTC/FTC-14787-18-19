package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.Collections;

@TeleOp(name="Mecanum Drive", group="Linear Opmode")
public class MecanumOpMode extends LinearOpMode {

    private final double DRIVE_POWER = 0.5;

    private ElapsedTime runtime;
    private boolean dPadUp;
    private boolean dPadDown;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        // Gamepad configuration and calculation variables
        double r, robotAngle, rightX, p1, p2, p3, p4;

        // Instantiate robot with hardware specifications
        RobotHardware robot = new RobotHardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
        robot.hang.setPower(-1);
        sleep(1000);
        */
        robot.deployment1.setPosition(0);
        robot.deployment2.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            dPadUp = gamepad1.dpad_up;
            dPadDown = gamepad1.dpad_down;

            p1 = r * Math.cos(robotAngle) - rightX;
            p2 = r * Math.sin(robotAngle) + rightX;
            p3 = r * Math.sin(robotAngle) - rightX;
            p4 = r * Math.cos(robotAngle) + rightX;

            robot.frontLeftDrive.setPower(p1);
            robot.frontRightDrive.setPower(p2);
            robot.backLeftDrive.setPower(p3);
            robot.backRightDrive.setPower(p4);

            if (dPadUp) {
                robot.hang.setPower(1);
            } else if (dPadDown) {
                robot.hang.setPower(-1);
            } else {
                robot.hang.setPower(0);
            }

            telemetry.addData("Status", "Runtime: %s", runtime.toString());
            telemetry.addData("Drive Train", "Front Left: %.2f\nFront Right: %.2f\nBack Left: %.2f\nBack Rigth: %.2f",
                    p1, p2, p3, p4);
            telemetry.update();
        }
    }
}
