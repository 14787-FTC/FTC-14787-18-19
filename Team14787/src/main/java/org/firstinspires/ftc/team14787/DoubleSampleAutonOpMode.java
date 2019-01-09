package org.firstinspires.ftc.team14787;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

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


        // Detect relative gold mineral position via recognizing the two far left minerals
        while (goldLocation == null) {
            goldLocation = vision.getGoldLocation();
        }

        // Deactive TFOD to preserve memory space
        vision.disableDetection();

        telemetry.addData("Status", "Gold location found, waiting for start");
        telemetry.addData("Gold Location", goldLocation);
        telemetry.update();

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

        // Fork behavior based off of detected gold position, eventually returns to center
        switch (goldLocation) {
            case LEFT: leftDetected(); break;
            case CENTER: centerDetected(); break;
            case RIGHT: rightDetected(); break;
        }

        rotate(35, ROTATE_MIN_POWER, ROTATE_MAX_POWER);
        strafeRight(9, STRAFE_POWER);
        moveForward(40, DRIVE_POWER);
        // Deployment1 = 0-0.35, Deployment2 = 1-0.65
        robot.deployment1.setPosition(0.35);
        moveBackward(3,DRIVE_POWER);
        rotate(-90, ROTATE_MIN_POWER, ROTATE_MAX_POWER);
        rotate(-90, ROTATE_MIN_POWER, ROTATE_MAX_POWER);

        // Enable on TFOD
        vision.enableDetection();
        sleep(250);

        int distance = 0;

        // Move slowly across minerals and find second gold position
        List<Recognition> updatedRecgonitions = vision.getRecognitions();
        telemetry.addData("Status", "Listing Percieve Minerals");
        if (updatedRecgonitions == null) telemetry.addData("Mineral", "Not found");
        else telemetry.addData("Mineral", updatedRecgonitions.get(0));
        telemetry.update();

        boolean run = updatedRecgonitions != null && !updatedRecgonitions.isEmpty() && updatedRecgonitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL);

        while (run && opModeIsActive()) {
            moveBackward(3, DRIVE_POWER);
            distance += 3;

            updatedRecgonitions = vision.getRecognitions();

            telemetry.addData("Status", "Listing Percieve Minerals");
            for (Recognition r : updatedRecgonitions) {
                if (r.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    run = false;
                }

                telemetry.addData("Mineral", r.getLabel());

            }
            telemetry.update();

            if (distance > 9) {
                run = false;
            }
        }

        // Knock off gold piece and return to crater using preserved distance value
        strafeRight(15, STRAFE_POWER);
        strafeLeft(12, STRAFE_POWER);
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
        moveBackward(3, DRIVE_POWER);
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
