package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Motor Testing", group="Autonomous")
public class TestingOpMode extends OpMode {
    private RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop() {
        robot.deployment2.setPosition(.65);

        telemetry.addData("Positional Status", "frontLeft: %d, frontRight: %d, backLeft: %d, backRight: %d", robot.frontLeftDrive.getCurrentPosition(), robot.frontRightDrive.getCurrentPosition(), robot.backLeftDrive.getCurrentPosition(), robot.backRightDrive.getCurrentPosition());
        telemetry.addData("Motor Status", "frontLeftDrive: %.2f, frontRightDrive: %.2f, backLeftDrive: %.2f, backRightDrive: %.2f", robot.frontLeftDrive.getPower(), robot.frontRightDrive.getPower(), robot.backLeftDrive.getPower(), robot.backRightDrive.getPower());
        telemetry.addData("Servo Status", "deployment1: %.1f, deployment2: %.1f", robot.deployment1.getPosition(), robot.deployment2.getPosition());
        telemetry.update();
    }
}
