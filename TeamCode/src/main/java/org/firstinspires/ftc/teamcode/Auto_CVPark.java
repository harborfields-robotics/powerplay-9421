package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CV.BarcodePositionDetector;

@Autonomous
public class Auto_CVPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();
        BigBird.cvUtil.init();

        BarcodePositionDetector.BarcodePosition barcodePosition;
        barcodePosition = BigBird.cvUtil.getBarcodePosition();
        telemetry.addData("Barcode position", barcodePosition);

        boolean one_dot = barcodePosition == BarcodePositionDetector.BarcodePosition.LEFT;
        boolean two_dot = barcodePosition == BarcodePositionDetector.BarcodePosition.MIDDLE;
        boolean three_dot = barcodePosition == BarcodePositionDetector.BarcodePosition.RIGHT;


  //      boolean one_dot = true; // for testing
    //    boolean two_dot = false; // for testing


        // start of program running

        // TO ADD: use CV to detect which zone to park in

		// return early so that it doesn't move
        double power = .4;
        BigBird.dt.driveForward(1.15, power); // Drive forward 1.125 tiles since all parking zones are 1 tile forwards
        BigBird.dt.setMotorPower(0,0,0,0);

        if (one_dot) {
            // move to left parking zone -- strafe left 1 tile
            BigBird.dt.strafeLeft(1, power);

        }
        else if (two_dot) {
            BigBird.dt.driveForward(0.1, power);
            // move to middle parking zone -- no extra movement
        }
        else {
            // move to right parking zone -- strafe right 1 tile
            BigBird.dt.strafeRight(1, power);
        }
        BigBird.dt.setMotorPower(0,0,0,0);

        Thread.sleep(2000);
    }
}
