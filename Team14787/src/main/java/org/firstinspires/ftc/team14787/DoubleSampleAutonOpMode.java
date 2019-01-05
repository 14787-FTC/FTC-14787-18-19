package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.team14787.Vision.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.team14787.Vision.LABEL_SILVER_MINERAL;

/**
 * Autonomous OpMode for double sample, starting at the crater
 */
@Autonomous(name="Double Sample OpMode", group="Autonomous")
public class DoubleSampleAutonOpMode extends AutonOpMode {

    /**
     * Run custom movements for this mode
     */
    @Override
    public void runOpMode() {
        super.runOpMode();

        telemetry.addData("Mode", "Knocking off gold piece");
        telemetry.update();

        // Fork behavior based off of detected gold position, eventually returns to center
        switch (goldLocation) {
            case LEFT: leftDetected(); break;
            case CENTER: centerDetected(); break;
            case RIGHT: rightDetected(); break;
        }

        rotate(33, ROTATE_MIN_POWER, ROTATE_MAX_POWER);
        strafeRight(9, STRAFE_POWER);
        moveForward(40, DRIVE_POWER);
        // Deployment1 = 0-0.35, Deployment2 = 1-0.65
        robot.deployment1.setPosition(0.35);
        moveBackward(3,DRIVE_POWER);
        rotate(-90, ROTATE_MIN_POWER, ROTATE_MAX_POWER);
        rotate(-90, ROTATE_MIN_POWER, ROTATE_MAX_POWER);

        // Enable on TFOD
        vision.enableDetection();

        int distance = 0;

        // Move slowly across minerals and find second gold position
        List<Recognition> updatedRecgonitions = vision.getUpdatedRecognitions();
        while (updatedRecgonitions.isEmpty() || (!updatedRecgonitions.isEmpty() && updatedRecgonitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL))) {
            moveBackward(9, DRIVE_POWER);
            distance += 9;
            updatedRecgonitions = vision.getUpdatedRecognitions();
        }

        // Knock off gold piece and return to crater using preserved distance value
        strafeRight(8, STRAFE_POWER);
        strafeLeft(8, STRAFE_POWER);
        moveForward(distance, DRIVE_POWER);
        rotate(45, ROTATE_MIN_POWER, ROTATE_MAX_POWER);
        strafeLeft(5, STRAFE_POWER);
        moveForward(87, .75);
    }

    /**
     * Movement fork methods based on goldLocation
     */
    private void leftDetected() {
        moveForward(5, DRIVE_POWER);
        strafeRight(12, STRAFE_POWER);
        strafeLeft(10, STRAFE_POWER);
        moveForward(25, DRIVE_POWER);
    }

    private void centerDetected() {
        moveBackward(5, DRIVE_POWER);
        strafeRight(12, STRAFE_POWER);
        strafeLeft(10, STRAFE_POWER);
        moveForward(40, DRIVE_POWER);
    }

    private void rightDetected() {
        moveBackward(20, DRIVE_POWER);
        strafeRight(10, STRAFE_POWER);
        strafeLeft(10, STRAFE_POWER);
        moveForward(48, DRIVE_POWER);
    }
}
