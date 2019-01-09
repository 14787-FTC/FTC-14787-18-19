package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Autonomous OpMode for single sample, starting at the crater
 */
@Autonomous(name="Depot Auton", group="Autonomous")
public class DepotAutonOpMode extends AutonOpMode {

    /**
     * Run custom movements for this mode
     */
    @Override
    public void runOpMode() {
        // Initial autonomous config/movement
        super.runOpMode();

        // Detect relative gold mineral position via recognizing the two far left minerals
        while (goldLocation == null) {
            goldLocation = vision.getGoldLocation();
        }

        // Deactive TFOD to preserve memory space
        vision.disableDetection();

        telemetry.addData("Status", "Gold location found, waiting for start");
        telemetry.addData("Gold Location", goldLocation);
        telemetry.update();

        sleep(2500);

        // Detach from hang and remove hook
        robot.hang.setPower(0);
        sleep(2000);
        robot.hang.setPower(0.5);
        sleep(500);
        robot.hang.setPower(0);
        telemetry.addData("Status", "Moving Forward");
        telemetry.update();
        //moveForward(8, 0.05);
        robot.setLeftPower(-0.5);
        robot.setRightPower(-0.3);
        sleep(275);
        robot.setDrivePower(0);
        strafeRight(13, STRAFE_POWER);

        telemetry.addData("Mode", "Knocking off gold piece");
        telemetry.update();

        // Fork behavior based off of detected gold position, eventually returns to equivocal position
        switch (goldLocation) {
            case LEFT: centerDetected(); break;
            case CENTER: leftDetected(); break;
            case RIGHT: rightDetected(); break;
        }

        /*
        rotate(40, ROTATE_MIN_POWER, ROTATE_MAX_POWER);
        strafeRight(9, STRAFE_POWER);
        moveForward(40, DRIVE_POWER);
        // Deployment1 = 0-0.35, Deployment2 = 1-0.65
        robot.deployment1.setPosition(0.35);
        moveBackward(72, DRIVE_POWER);
        */
    }

    /**
     * Forked movement methods based on goldLocation
     */
    private void leftDetected() {
        moveForward(5, DRIVE_POWER);
        strafeRight(14, STRAFE_POWER);
        strafeLeft(12, STRAFE_POWER);
        //moveForward(25, DRIVE_POWER);
    }

    private void centerDetected() {
        moveBackward(5, DRIVE_POWER);
        strafeRight(12, STRAFE_POWER);
        strafeLeft(10, STRAFE_POWER);
        //moveForward(40, DRIVE_POWER);
    }

    private void rightDetected() {
        moveBackward(20, DRIVE_POWER);
        strafeRight(10, STRAFE_POWER);
        strafeLeft(10, STRAFE_POWER);
        //moveForward(48, DRIVE_POWER);
    }
}
