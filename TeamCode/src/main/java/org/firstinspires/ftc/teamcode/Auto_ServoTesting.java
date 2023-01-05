package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Auto_ServoTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();

        while (opModeIsActive()) {
            BigBird.elbow1.setPosition(1);
            BigBird.elbow2.setPosition(1);
            Thread.sleep(1500);
            BigBird.elbow1.setPosition(0);
            BigBird.elbow2.setPosition(0);
            Thread.sleep(1500);
        }

    }

}
