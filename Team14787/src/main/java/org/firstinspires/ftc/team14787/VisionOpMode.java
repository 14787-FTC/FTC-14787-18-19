package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Timer;

/**
 * Autonomous OpMode which simply enables vision detection
 */
@Autonomous(name="Vision Testing", group="Autonomous")
public class VisionOpMode extends LinearOpMode {

    /** Robot representation */
    private RobotHardware robot;

    private boolean finishedTime;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        //robot.hang.setPower(-1);
        sleep(2500);

        Vision vision = new Vision(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        MineralLocation craterLocation = null;
        MineralLocation depotLocation = null;

        // Detect relative gold mineral position via recognizing the two far left minerals
        while (craterLocation == null && depotLocation == null && opModeIsActive()) {
            craterLocation = vision.getGoldLocation();
        }

        telemetry.addData("craterLocation", craterLocation);
        telemetry.update();

        //robot.hang.setPower(0);

        while (opModeIsActive()) {
            sleep(1000);
        }

        vision.disableDetection();
    }
}
