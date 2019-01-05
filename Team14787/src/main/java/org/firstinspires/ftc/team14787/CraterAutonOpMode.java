package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Crater Auton", group="Autonomous")
public class CraterAutonOpMode extends AutonOpMode {

    private final double STRAFE_POWER = 0.1;

    @Override
    public void runOpMode() {
        super.runOpMode();

        robot.hang.setPower(0);
        sleep(2000);
        robot.hang.setPower(1);
        sleep(500);
        robot.hang.setPower(0);

        telemetry.addData("Mode", "Knocking off gold piece");
        telemetry.update();

        moveForward(8, 0.05);
        strafeRight(13, STRAFE_POWER);

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
        moveBackward(87, DRIVE_POWER);

        while (opModeIsActive()) {
            sleep(1000);
        }
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
