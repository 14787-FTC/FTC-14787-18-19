package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous OpMode which simply enables vision detection
 */
@Autonomous(name="Vision Testing", group="Autonomous")
public class VisionOpMode extends LinearOpMode {

    /** Robot representation */
    private RobotHardware robot;

    @Override
    public void runOpMode() {
        //RobotHardware robot = new RobotHardware(hardwareMap);
        //robot.hang.setPower(-1);

        Vision vision = new Vision(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        MineralLocation goldLocation;

        // Detect relative gold mineral position via recognizing the two far left minerals
        while (opModeIsActive()) {
            vision.getGoldLocation();
        }

        vision.disableDetection();
    }
}
