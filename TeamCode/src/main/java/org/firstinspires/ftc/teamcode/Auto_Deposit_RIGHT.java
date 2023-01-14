package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Auto_Deposit_RIGHT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();
        // return early so that it doesn't move
        double power = .4;

        BigBird.slides.setTargetPosition(SlidesTarget.FRONT_HIGH.slides_position); // Move the slides to the highest target
        BigBird.flipElbowAndWrist(true);
        Thread.sleep(500);

        BigBird.dt.driveForward(1, power);
        BigBird.dt.strafeRight(0.5, 0.2);

        Thread.sleep(500); // Wait .5s for the cone to fall
        BigBird.grabber.openClaw();
        Thread.sleep(500);
        BigBird.flipElbowAndWrist(false);
        BigBird.slides.setTargetPosition(SlidesTarget.FRONT_GROUND.slides_position);
        BigBird.dt.strafeLeft(0.5, 0.2);
        BigBird.dt.driveBackward(1, power);
        Thread.sleep(250);
        BigBird.dt.strafeLeft(0.5, 0.2);
        BigBird.dt.setMotorPower(0,0,0,0);

        Thread.sleep(2000);
    }
}
