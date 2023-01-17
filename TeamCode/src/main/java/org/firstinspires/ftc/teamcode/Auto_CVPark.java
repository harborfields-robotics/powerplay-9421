package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CV.BarcodePositionDetector;

@Autonomous
public class Auto_CVPark extends LinearOpMode {
    public static final boolean TESTING = false;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        if (!TESTING)
            BigBird.init();
        waitForStart();
        // return early so that it doesn't move
        double power = .4;
        BigBird.dt.driveForward(0.8, power); // Drive forward 1.125 tiles since all parking zones are 1 tile forwards
        BigBird.dt.strafeRight(0.15, 0.3);
        Thread.sleep(250);

        BigBird.cvUtil.init();

        BarcodePositionDetector.BarcodePosition barcodePosition;
        barcodePosition = BigBird.cvUtil.getBarcodePosition();
        telemetry.addData("Barcode position", barcodePosition);

        boolean one_dot = barcodePosition == BarcodePositionDetector.BarcodePosition.LEFT; // blue
        boolean two_dot = barcodePosition == BarcodePositionDetector.BarcodePosition.MIDDLE; // green
        boolean three_dot = barcodePosition == BarcodePositionDetector.BarcodePosition.RIGHT; // red

        // start of program running

        // TO ADD: use CV to detect which zone to park in

		// return early so that it doesn't move
        double power = 0.4;
        BigBird.dt.driveForward(1.15, power); // Drive forward 1.125 tiles since all parking zones are 1 tile forwards
        BigBird.dt.setMotorPower(0, 0, 0, 0);

        if (!TESTING) {
            BigBird.dt.strafeLeft(0.15, 0.3);
            BigBird.dt.driveForward(0.25, power);
            Thread.sleep(250);
        }
        if (one_dot) {
            // move to left parking zone -- strafe left 1 tile
            BigBird.dt.strafeLeft(1, power);
        } else if (two_dot) {
            // move to middle parking zone -- no extra movement
            BigBird.dt.driveForward(0.1, power);
        } else {
            // move to right parking zone -- strafe right 1 tile
            BigBird.dt.strafeRight(1, power);
        }
        BigBird.dt.setMotorPower(0,0,0,0);

        Thread.sleep(2000);
    }
}
