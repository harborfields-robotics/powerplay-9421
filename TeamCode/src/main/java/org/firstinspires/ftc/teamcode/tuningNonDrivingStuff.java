package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Hardware.elbow_min;
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
public class tuningNonDrivingStuff extends LinearOpMode {

    public double clamp(double x, double min, double max) {
        return x > max ? max : (Math.max(x, min));
    }

    static boolean elbowToggleMode = true;
    static boolean slidesToggleMode = true;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        Drivetrain dt = BigBird.getDrivetrain();

        BigBird.init();
        waitForStart();

        //DrivetrainSpeedUpdateThread driveSpeed = new DrivetrainSpeedUpdateThread(BigBird, dt.FL, dt.BL, dt.FR, dt.BR);
        //driveSpeed.start();

        SlidesThread slidesThread = new SlidesThread(BigBird);
        slidesThread.start();

        GrabberThread grabberThread = new GrabberThread(BigBird);
        grabberThread.start();

        while (opModeIsActive()) {

            telemetry.addData("claw position: ", BigBird.grabber.claw.getPosition());
            telemetry.addData("wrist position: ", BigBird.grabber.wrist.getPosition());
            telemetry.addData("slides: ", BigBird.slides.getCurrentPosition());
            telemetry.addData("elbow: ", BigBird.elbow1.getPosition());
            telemetry.addData("Slides toggle mode", slidesToggleMode);
            telemetry.addData("Elbow toggle mode", elbowToggleMode);

            telemetry.update();
        }
    }

    public class SlidesThread extends Thread {
        Hardware BigBird;
        DcMotor slides;
        SlidesTarget slidesPosition = null;
        boolean manual_slides = false;

        public SlidesThread(Hardware BigBird) {
            this.BigBird = BigBird;
            slides = BigBird.slides;
            slidesPosition = Hardware.slidesPosition;
        }

        public void run() {
            boolean front = true;
            boolean elbow_changed = false;
            boolean elbow_mode_changed = false;
            boolean slides_mode_changed = false;
            elbowToggleMode = false;
            slidesToggleMode = false;

            while (opModeIsActive()) {

                // swap between toggle mode and manual mode
                /*
                if (gamepad1.start && !elbow_mode_changed) {
                    elbowToggleMode = !elbowToggleMode;
                    elbow_mode_changed = true;
                }
                else if (!gamepad1.start) {
                    elbow_mode_changed = false;
                }
                 */

                // toggle elbow controls
                if (elbowToggleMode) {
                    if (gamepad1.a && !elbow_changed) {
                        if (front) {
                            try {BigBird.flipElbowAndWrist(true);} catch (InterruptedException ignored) {}
                            front = false;
                        }
                        else {
                            try {BigBird.flipElbowAndWrist(false);} catch (InterruptedException ignored) {}
                            front = true;
                        }
                        elbow_changed = true;
                    }
                    else if (!gamepad1.a) {
                        elbow_changed = false;
                    }
                }

                // manual elbow controls
                else {
                    if (gamepad1.right_stick_y > 0.3) {
                        BigBird.elbow1.setPosition(Math.min(1, BigBird.elbow1.getPosition() + .01));
                        try {Thread.sleep(25);} catch (InterruptedException e) {}
                    }
                    else if (gamepad1.right_stick_y < -0.3) {
                        BigBird.elbow1.setPosition(Math.max(BigBird.elbow1.getPosition() - .01, 0));
                        try {Thread.sleep(25);} catch (InterruptedException e) {}
                    }
                }

                // swap between toggle and manual slides controls
                /*
                if (gamepad1.back && !slides_mode_changed) {
                    slidesToggleMode = !slidesToggleMode;
                    slides_mode_changed = true;
                }
                else if (!gamepad1.start) {
                    slides_mode_changed = false;
                }
                 */

                // toggle slides controls
                if (slidesToggleMode) {
                    if (gamepad1.dpad_up) {
                        slides.setTargetPosition(SlidesTarget.BACK_HIGH.slides_position);
                    }
                    if (gamepad1.dpad_left) {
                        slides.setTargetPosition(SlidesTarget.BACK_MIDDLE.slides_position);
                    }
                    if (gamepad1.dpad_right) {
                        slides.setTargetPosition(SlidesTarget.BACK_LOW.slides_position);
                    }
                    if (gamepad1.dpad_down) {
                        slides.setTargetPosition(SlidesTarget.FRONT_GROUND.slides_position);
                    }
                }

                // manual slides controls
                else {
                    if (gamepad1.left_stick_y > 0.3) {
                        slides.setTargetPosition(Math.min(slides.getCurrentPosition() + 1, SlidesTarget.BACK_HIGH.slides_position));
                        try {Thread.sleep(10);} catch (InterruptedException e) {}
                    }
                    else if (gamepad1.left_stick_y < -0.3) {
                        slides.setTargetPosition(Math.max(slides.getCurrentPosition() - 1, 0));
                        try {Thread.sleep(10);} catch (InterruptedException e) {}
                    }
                }

                if (slides.getCurrentPosition() > slides.getTargetPosition()) {
                    BigBird.slides.setPower(.8);
                }
                else if (BigBird.slides.getCurrentPosition() < slides.getTargetPosition()) {
                    BigBird.slides.setPower(1);
                }
                else if (BigBird.slides.getCurrentPosition() == slides.getTargetPosition()) {
                    BigBird.slides.setPower(0);
                }
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
                    if (grabber.isClosed) {
                        grabber.openClaw();
                    } else {
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
                    } else {
                        grabber.flipGrabberFace();
                    }
                    wrist_changed = true;
                } else if (!gamepad1.y) {
                    wrist_changed = false;
                }


            }
        }
    }


    public void adjustMotorPower(DcMotor motor, double targetSpeed, double speedIncrement) {
        double speed = motor.getPower();
        if (speed > targetSpeed) {
            motor.setPower((speed > targetSpeed + speedIncrement) ? speed - speedIncrement : targetSpeed);
        } else if (speed < targetSpeed) {
            motor.setPower((speed < targetSpeed - speedIncrement) ? speed + speedIncrement : targetSpeed);
        }
    }

    public void setMotorPowers(Hardware BigBird, double FLspeed, double BLspeed, double FRspeed, double BRspeed) {
        BigBird.dt.FL.setPower(FLspeed);
        BigBird.dt.FR.setPower(FRspeed);
        BigBird.dt.BL.setPower(BLspeed);
        BigBird.dt.BR.setPower(BRspeed);
    }

    public void resetSlides(Hardware BigBird) {
        BigBird.slides.setTargetPosition(SlidesTarget.FRONT_GROUND.slides_position);
        BigBird.elbow1.setPosition(SlidesTarget.FRONT_GROUND.elbow_position);
        BigBird.elbow2.setPosition(SlidesTarget.FRONT_GROUND.elbow_position);
    }


}