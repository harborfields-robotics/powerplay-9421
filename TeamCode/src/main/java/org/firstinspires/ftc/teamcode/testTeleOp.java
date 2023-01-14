package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Hardware.slidesPosition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//hardware
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import kotlin.ranges.ClosedFloatingPointRange;


/* As of Sunday, 12/11/2022
 * TO-DO:
 * - slides code (motor + elbow servos)
 * - field centric controls (tutorial on gm0)
 * - consolidate slides motor and elbow servos under Slides object
 *
 * IN PROGRESS:
 *
 * COMPLETE:
 * - smooth acceleration robot centric drive
 * - temporary slides code (1 motor)
 * - set servos to 0 on initialization
 */


@TeleOp
public class testTeleOp extends LinearOpMode {

    public double clamp(double x, double min, double max) {
        return x > max ? max : (Math.max(x, min));
    }

    static boolean smooth_controls = true;
    static boolean constant_controls = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        Drivetrain dt = BigBird.getDrivetrain();

        BigBird.init();
        waitForStart();

        DrivetrainSpeedUpdateThread driveSpeed = new DrivetrainSpeedUpdateThread(BigBird, dt.FL, dt.BL, dt.FR, dt.BR);
        driveSpeed.start();

        SlidesThread slidesThread = new SlidesThread(BigBird);
        slidesThread.start();

        GrabberThread grabberThread = new GrabberThread(BigBird);
        grabberThread.start();

        while (opModeIsActive()) {

            telemetry.addData("claw position: ", BigBird.grabber.claw.getPosition());
            telemetry.addData("wrist position: ", BigBird.grabber.wrist.getPosition());
            telemetry.addData("slides: ", BigBird.slides.getCurrentPosition());
            telemetry.addData("elbow: ", BigBird.elbow1.getPosition());

            telemetry.update();
        }
    }

    /*
     * TO-DO: make independent thread to operate claw, or otherwise make it work
    public class ClawOperatorThread extends Thread {

        Hardware BigBird;
        public ClawOperatorThread(Hardware BigBird) {

        }

        @Override
        public void run() {
            if (BigBird.grabber.claw.getPosition() == )
        }
    }

     */

    public class DrivetrainSpeedUpdateThread extends Thread {

        Hardware BigBird;
        DcMotor FL;
        DcMotor BL;
        DcMotor FR;
        DcMotor BR;

        public DrivetrainSpeedUpdateThread(Hardware BigBird, DcMotor m1, DcMotor m2, DcMotor m3, DcMotor m4) {
            this.BigBird = BigBird;
            FL = m1;
            BL = m2;
            FR = m3;
            BR = m4;
        }

        @Override
        public void run() {
            double speedIncrement = .05;

            while (opModeIsActive()) {

                // smooth controls
                if (gamepad1.left_bumper) {
                    smooth_controls = true;
                    constant_controls = false;
                }
                // default control mode
                else if (gamepad1.right_bumper) {
                    smooth_controls = false;
                    constant_controls = false;
                }
                // "block" controls
                else if (gamepad1.start) {
                    smooth_controls = false;
                    constant_controls = true;
                }

                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;
                double slow = .2;

                if (gamepad1.left_trigger > .3) {
                    y *= slow;
                    x *= slow;
                    rx *= slow;
                }

                double FLTargetSpeed = clamp(y + x + rx, -1, 1);
                double BLTargetSpeed = clamp(y - x + rx, -1, 1);
                double FRTargetSpeed = clamp(y - x - rx, -1, 1);
                double BRTargetSpeed = clamp(y + x - rx, -1, 1);

                if (smooth_controls) { // Smooth acceleration & deceleration
                    adjustMotorPower(FL, FLTargetSpeed, speedIncrement);
                    adjustMotorPower(BL, BLTargetSpeed, speedIncrement);
                    adjustMotorPower(FR, FRTargetSpeed, speedIncrement);
                    adjustMotorPower(BR, BRTargetSpeed, speedIncrement);
                    try {Thread.sleep(5);} catch (InterruptedException e) {e.printStackTrace();}
                }
                else if (constant_controls) { // Move one tile in a cardinal direction
                    if (gamepad1.dpad_up) {
                        try {BigBird.dt.driveForward(1, .8);} catch (InterruptedException e) {}
                    }
                    else if (gamepad1.dpad_down) {
                        try {BigBird.dt.driveBackward(1, .8);} catch (InterruptedException e) {}
                    }
                    else if (gamepad1.dpad_left) {
                        try {BigBird.dt.strafeLeft(1, .8);} catch (InterruptedException e) {}
                    }
                    else if (gamepad1.dpad_right) {
                        try {BigBird.dt.strafeRight(1, .8);} catch (InterruptedException e) {}
                    }
                }
                else { // Normal controls
                    setMotorPowers(BigBird, FLTargetSpeed, BLTargetSpeed, FRTargetSpeed, BRTargetSpeed);
                }
            }
        }
    }

    public class SlidesThread extends Thread {
        Hardware BigBird;
        DcMotor slides;
        SlidesTarget slidesPosition = null;

        public SlidesThread(Hardware BigBird) {
            this.BigBird = BigBird;
            slides = BigBird.slides;
            slidesPosition = Hardware.slidesPosition;
        }

        public void run() {
            while (opModeIsActive()) {
                if (gamepad1.dpad_up) {
                    slidesPosition = SlidesTarget.FRONT_HIGH;
                }
                else if (gamepad1.dpad_left || gamepad1.dpad_right) {
                    slidesPosition = SlidesTarget.FRONT_MIDDLE;
                }
                else if (gamepad1.dpad_down) {
                    slidesPosition = (slidesPosition == SlidesTarget.FRONT_LOW) ? SlidesTarget.FRONT_GROUND : SlidesTarget.FRONT_LOW;
                }
                if (BigBird.slides.getCurrentPosition() > slidesPosition.slides_position) {
                    BigBird.slides.setPower(.8);
                }
                else if (BigBird.slides.getCurrentPosition() < slidesPosition.slides_position) {
                    BigBird.slides.setPower(1);
                }
                else if (BigBird.slides.getCurrentPosition() == slidesPosition.slides_position) {
                    BigBird.slides.setPower(0);
                }
                BigBird.slides.setTargetPosition(slidesPosition.slides_position);
                BigBird.elbow1.setPosition(slidesPosition.elbow_position);
                BigBird.elbow2.setPosition(slidesPosition.elbow_position);
            }
        }

    }

    public class GrabberThread extends Thread {
        Hardware BigBird;
        Grabber grabber;

        public GrabberThread(Hardware BigBird) {
            this.BigBird = BigBird;
            grabber = BigBird.grabber;
        }

        public void run() {
            boolean claw_changed = false; //Outside of loop()
            boolean wrist_changed = false;
            while (opModeIsActive()) {

                // Toggle x button to open/close claw
                if (gamepad1.x && !claw_changed) {
                    if(grabber.isClosed) {
                        grabber.openClaw();
                    }
                    else {
                        grabber.closeClaw();
                    }
                    claw_changed = true;
                } else if (!gamepad1.x) {
                    claw_changed = false;
                }

                // Toggle y button to flip grabber
                if (gamepad1.y && !wrist_changed) {
                    if (grabber.isFlipped) {
                        grabber.rightGrabberFace();
                    }
                    else {
                        grabber.flipGrabberFace();
                    }
                    wrist_changed = true;
                } else if (!gamepad1.y) {
                    wrist_changed = false;
                }


            }
        }
    }


    public static void adjustMotorPower(DcMotor motor, double targetSpeed, double speedIncrement) {
        double speed = motor.getPower();
        if (speed > targetSpeed) {
            motor.setPower((speed > targetSpeed + speedIncrement) ? speed - speedIncrement : targetSpeed);
        } else if (speed < targetSpeed) {
            motor.setPower((speed < targetSpeed - speedIncrement) ? speed + speedIncrement : targetSpeed);
        }
    }

    public static void setMotorPowers(Hardware BigBird, double FLspeed, double BLspeed, double FRspeed, double BRspeed) {
        BigBird.dt.FL.setPower(FLspeed);
        BigBird.dt.FR.setPower(FRspeed);
        BigBird.dt.BL.setPower(BLspeed);
        BigBird.dt.BR.setPower(BRspeed);
    }

    public static void resetSlides(Hardware BigBird) {
        BigBird.slides.setTargetPosition(SlidesTarget.FRONT_GROUND.slides_position);
        BigBird.elbow1.setPosition(SlidesTarget.FRONT_GROUND.elbow_position);
        BigBird.elbow2.setPosition(SlidesTarget.FRONT_GROUND.elbow_position);
    }
}