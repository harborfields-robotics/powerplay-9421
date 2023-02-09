package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class Auto_5CycleRight extends LinearOpMode {

    static int postDepositWaitTime = 200; // time between dropping cone and moving robot again (milliseconds)
    static double turnPower = 0.35;
    static double drivePower = 0.6;
    static double strafePower = 0.6;
    static double topConePosition = 0.93;
    static int junctionOffsetAngle = 15;
    static int pauseTime = 100;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        CycleConeThread cct = new CycleConeThread(BigBird,true, SlidesTarget.FRONT_GROUND.slides_position);

        BigBird.init();
        waitForStart();

        /*
         * Tiles are notated like on the coordinate grid.
         * - bottom left tile is [1, 1]
         * - bottom right tile is [6, 1]
         * - top left tile is [1, 6]
         * - top right tile is [6, 6]
         *
         * [x, y] - means center of tile
         * [x1, y1] b [x2, y2] means between two tiles
         */

        // 1. Use camera on sleeve to find final parking position

        // 2. Navigate: [5, 1] --> [5, 2]
        BigBird.flipElbowAndWrist(true, Hardware.elbow_min); // flip elbow up to target position
        BigBird.slides.setTargetPosition(SlidesTarget.BACK_HIGH.slides_position); // extend slides to target position
        BigBird.slides.setPower(1);

        BigBird.dt.driveBackward(1.5, drivePower);
        Thread.sleep(pauseTime);

        // 3. Turn to face back towards right high junction
        BigBird.dt.turnLeft(40, turnPower);
        Thread.sleep(pauseTime);

        // 4. Deposit preloaded cone
        Thread.sleep(300);
        BigBird.grabber.openClaw(); // drop cone
        Thread.sleep(250);

        // 5. Navigate to stack: [5, 2] --> [5, 3] --> [6, 3] --> precise position
        BigBird.slides.setTargetPosition(0); // extend slides to target position
        BigBird.dt.turnRight(30, turnPower);
        Thread.sleep(pauseTime);
        BigBird.flipElbowAndWrist(false, topConePosition);
        Thread.sleep(pauseTime);
        BigBird.dt.driveBackward(0.65, drivePower);
        Thread.sleep(pauseTime);

        BigBird.dt.turnLeft(90, turnPower);
        Thread.sleep(pauseTime);
        BigBird.dt.driveForward(0.8, drivePower);
        Thread.sleep(pauseTime);
        BigBird.dt.strafeLeft(0.25, strafePower);
        Thread.sleep(pauseTime);
        //BigBird.dt.turnRight(junctionOffsetAngle, drivePower);
        //Thread.sleep(pauseTime);
        BigBird.dt.driveForward(0.3, drivePower);
        Thread.sleep(pauseTime);
        BigBird.grabber.closeClaw();

        // 6. Do 5 cycles
        for (double grabberAngle = topConePosition; grabberAngle <= 1.00; grabberAngle += (1.00-topConePosition) / 4.0) {
            BigBird.autoDeposit(SlidesTarget.BACK_HIGH.slides_position + 300, SlidesTarget.BACK_HIGH.elbow_position - .08, grabberAngle);
            BigBird.grabber.closeClaw();
            Thread.sleep(300);
        }
        // 7. Park in designated zone: [6, 3] --> signal zone
        BigBird.dt.driveBackward(0.15, drivePower);
        Thread.sleep(pauseTime);
        BigBird.dt.turnLeft(junctionOffsetAngle, drivePower);
        Thread.sleep(pauseTime);
        BigBird.dt.driveBackward(0.15, drivePower);

        // - Left zone (one dot): [2, 3] b [3, 3] --> [1, 3]
        if (false) {
            BigBird.dt.driveBackward(2.15, drivePower);
        }
        // - Middle zone (one dot): [2, 3] b [3, 3] --> [2, 3]
        else if (false) {
            BigBird.dt.driveBackward(1.15, drivePower);
        }
        // - Right zone (one dot): [2, 3] b [3, 3] --> [3, 3]
        else if (false) {
            //BigBird.dt.strafeRight(0.5, strafePower);
        }

    }

    // unused as of 1/25/23
    public static class DriveThread extends Thread {

        Hardware BigBird = null;
        Drivetrain dt = null;
        DcMotor FL = null;
        DcMotor BL = null;
        DcMotor FR = null;
        DcMotor BR = null;

        public DriveThread(Hardware bb) {
            BigBird = bb;
            dt = bb.dt;
            FL = bb.dt.FL;
            BL = bb.dt.BL;
            FR = bb.dt.FR;
            BR = bb.dt.BR;
        }

        @Override
        public void run() {

        }

    }

    public static class CycleConeThread extends Thread {

        Hardware BigBird = null;
        SlidesTarget slidesPosition = SlidesTarget.FRONT_GROUND;
        volatile boolean waiting;
        int coneDropDelay = 0; // time between slides reaching target position and cone being dropped (milliseconds)
        static double slidesPowerUp = 0.8;
        static double slidesPowerDown = 0.6;
        int slidesResetPosition; // final slides position of the cycle, set in initialization

        public CycleConeThread(Hardware b, boolean w, int srp) {
            BigBird = b;
            waiting = w;
            slidesResetPosition = srp;
        }

        @Override
        public void run() {
            // move slides up, flip elbow and wrist
            try {BigBird.flipElbowAndWrist(true);} catch (InterruptedException ignored) {}
            BigBird.slides.setTargetPosition(SlidesTarget.BACK_HIGH.slides_position);
            while (BigBird.slides.getCurrentPosition() < BigBird.slides.getTargetPosition()) {
                BigBird.slides.setPower(slidesPowerUp);
            }
            BigBird.slides.setPower(0);

            while (waiting) {} // wait for cue from drive thread to drop cone
            BigBird.grabber.openClaw(); // drop cone
            try {Thread.sleep(postDepositWaitTime);} catch (InterruptedException e) {} // wait a bit for the cone to drop before proceeding

            // go back down and flip back to front
            try {BigBird.flipElbowAndWrist(false);} catch (InterruptedException ignored) {}
            BigBird.slides.setTargetPosition(slidesResetPosition);
            while (BigBird.slides.getCurrentPosition() > slidesResetPosition) {
                BigBird.slides.setPower(slidesPowerDown);
            }
            BigBird.slides.setPower(0);
        }

        public void changeSlidesPosition() {
            boolean increases = BigBird.slides.getCurrentPosition() < slidesPosition.slides_position;
            BigBird.slides.setPower(0);
        }
    }
}
