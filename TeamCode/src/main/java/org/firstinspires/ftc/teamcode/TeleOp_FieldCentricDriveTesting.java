package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/* Class objective:
 * - isolate formula for field centric controls to test them
 * - sampled from GM0
 */

@TeleOp

public class TeleOp_FieldCentricDriveTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        // Technically this is the default, but specifying it is clearer

        while (opModeIsActive()) {

            double y  = gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x * 1.1; // accounts for imperfect strafing, allegedly
            double rx = gamepad1.right_stick_x;

            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            // Without this, data retrieving from the IMU throws an exception

            // read inverse IMU heading, as the IMU heading is clockwise positive and needs to be negated
            double botHeading = -imu.getAngularOrientation().firstAngle;

            /* Get rotational components of x and y using IMU angle
             * x2 = x1cosB - y1sinB
             * y2 = x1sinB - y1cosB
             * B is the angle to rotate by
             * x1 and y1 are the components of the original vector (original joystick positions)
             * x2 and y2 are the components of the resultant vector
             */

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) - y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            // If left trigger is held, multiply denominator to activate slow drive
            if (gamepad1.left_trigger >= .5) {
                denominator *= 5;
            }
            double FLpower = (rotY + rotX + rx) / denominator;
            double BLpower = (rotY - rotX + rx) / denominator;
            double FRpower = (rotY - rotX - rx) / denominator;
            double BRpower = (rotY + rotX - rx) / denominator;

            BigBird.dt.setMotorPower(FLpower, BLpower, FRpower, BRpower);
        }

    }
}
