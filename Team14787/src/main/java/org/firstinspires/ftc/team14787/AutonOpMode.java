package org.firstinspires.ftc.team14787;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="AutonOpMode", group="Autonomous")
public class AutonOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Configure IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode           = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "Calibrating IMU");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Waiting for start");
        telemetry.addData("IMU Calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();

        RobotHardware robot = new RobotHardware(telemetry, hardwareMap, imu);
        waitForStart();

        // Move forward for 250 milliseconds
        /*
        for (DcMotor motor : robot.driveTrain) {
            motor.setPower(1);
        }
        sleep(250);
        */

        telemetry.addData("Mode", "Turning left");
        telemetry.update();

        robot.right(90, .4);
    }
}
