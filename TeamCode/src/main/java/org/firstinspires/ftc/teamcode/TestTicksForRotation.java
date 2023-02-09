package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestTicksForRotation extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();

        BigBird.dt.turnLeft(360, 0.5);
    }

}
