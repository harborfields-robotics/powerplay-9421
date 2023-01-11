package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Auto_CVPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();

        boolean one_dot = false;
        boolean two_dot = false;
        boolean three_dot = false;

        // start of program running

        // TO ADD: use CV to detect which zone to park in

        BigBird.dt.driveForward(1.0,0.5); // Drive forward 1 tile since all parking zones are 1 tile forwards



        if (one_dot) {
            // move to left parking zone -- strafe left 1 tile
            BigBird.dt.strafeLeft(1.0, 0.5);

        }
        else if (two_dot) {
            // move to middle parking zone -- no extra movement
        }
        else {
            // move to right parking zone -- strafe right 1 tile
            BigBird.dt.strafeRight(1.0, 0.5);
        }

    }
}
