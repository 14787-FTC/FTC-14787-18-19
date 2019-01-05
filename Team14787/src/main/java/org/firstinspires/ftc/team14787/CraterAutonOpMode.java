package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Autonomous OpMode for single sample, starting at the crater
 */
@Autonomous(name="Crater Auton", group="Autonomous")
public class CraterAutonOpMode extends AutonOpMode {

    /**
     * Run custom movements for this mode
     */
    @Override
    public void runOpMode() {
        // Initial autonomous config/movement
        super.runOpMode();

        telemetry.addData("Mode", "Knocking off gold piece");
        telemetry.update();

        // Fork behavior based off of detected gold position, eventually returns to equivocal position
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
        moveBackward(87, DRIVE_POWER);
    }

    /**
     * Forked movement methods based on goldLocation
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
