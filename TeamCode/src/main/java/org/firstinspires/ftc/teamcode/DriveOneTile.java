package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DriveOneTile extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();
        BigBird.dt.driveForward(1.0, 0.4);
    }
}
