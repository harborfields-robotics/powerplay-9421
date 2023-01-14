package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Auto_Deposit_LEFT extends LinearOpMode {
    public double fwdDistance = 2.6;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();
        // return early so that it doesn't move
        double power = .4;


        BigBird.slides.setTargetPosition(SlidesTarget.FRONT_HIGH.slides_position); // Move the slides to the highest target
        BigBird.dt.driveForward(fwdDistance, power);
        Thread.sleep(1000);

        BigBird.dt.strafeRight(0.3, .8);
        Thread.sleep(1000);

        BigBird.slides.setTargetPosition(SlidesTarget.FRONT_HIGH.slides_position); // Move the slides to the highest target

        Thread.sleep(1000);

        BigBird.flipElbowAndWrist(true);

        Thread.sleep(2000);


        BigBird.grabber.openClaw();
        Thread.sleep(500);
        BigBird.flipElbowAndWrist(false);
        Thread.sleep(500);

        BigBird.slides.setTargetPosition(SlidesTarget.FRONT_GROUND.slides_position);
        BigBird.dt.strafeLeft(0.5, 0.2);
        BigBird.dt.driveBackward(fwdDistance, power);
        Thread.sleep(250);
        BigBird.dt.strafeLeft(0.5, 0.2);
        BigBird.dt.setMotorPower(0,0,0,0);

        Thread.sleep(2000);
    }
}
