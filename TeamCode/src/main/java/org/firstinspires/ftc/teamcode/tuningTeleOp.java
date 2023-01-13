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
public class tuningTeleOp extends LinearOpMode {

    public double clamp(double x, double min, double max) {
        return x > max ? max : (Math.max(x, min));
    }

    static boolean smooth_controls = true;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        Drivetrain dt = BigBird.getDrivetrain();

        //BigBird.init();
        BigBird.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        DrivetrainSpeedUpdateThread driveSpeed = new DrivetrainSpeedUpdateThread(BigBird, dt.FL, dt.BL, dt.FR, dt.BR);
        driveSpeed.start();

        //SlidesThread slidesThread = new SlidesThread(BigBird);
        //slidesThread.start();

        GrabberThread grabberThread = new GrabberThread(BigBird);
        grabberThread.start();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && BigBird.slides.getCurrentPosition() < SlidesTarget.FRONT_HIGH.slides_position) {
                BigBird.slides.setTargetPosition(BigBird.slides.getTargetPosition()+1);
            }
            else if (gamepad1.dpad_down && BigBird.slides.getCurrentPosition() > 0) {
                BigBird.slides.setTargetPosition(BigBird.slides.getTargetPosition()-1);
            }

            if (gamepad1.dpad_left && BigBird.elbow1.getPosition() > 0 && BigBird.elbow2.getPosition() > 0) {
                BigBird.elbow1.setPosition(BigBird.elbow1.getPosition() - .01);
                BigBird.elbow2.setPosition(BigBird.elbow2.getPosition() - .01);
            }
            else if(gamepad1.dpad_right && BigBird.elbow1.getPosition() < 1 && BigBird.elbow2.getPosition() < 1) {
                BigBird.elbow1.setPosition(BigBird.elbow2.getPosition() + .01);
                BigBird.elbow2.setPosition(BigBird.elbow1.getPosition() + .01);
            }

            if (gamepad1.left_bumper) {
                smooth_controls = true;
            }
            else if (gamepad1.right_bumper) {
                smooth_controls = false;
            }

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

                if (smooth_controls) {
                    adjustMotorPower(FL, FLTargetSpeed, speedIncrement);
                    adjustMotorPower(BL, BLTargetSpeed, speedIncrement);
                    adjustMotorPower(FR, FRTargetSpeed, speedIncrement);
                    adjustMotorPower(BR, BRTargetSpeed, speedIncrement);
                }
                else {
                    setMotorPowers(BigBird, FLTargetSpeed, BLTargetSpeed, FRTargetSpeed, BRTargetSpeed);
                }

                try {Thread.sleep(5);} catch (InterruptedException e) {e.printStackTrace();}
            }
        }
    }

    public class SlidesThread extends Thread {
        Hardware BigBird;
        DcMotor slides;
        SlidesTarget slidesPosition = null;
        int slides_encoder_target = 0;

        public SlidesThread(Hardware BigBird) {
            this.BigBird = BigBird;
            slides = BigBird.slides;
            slidesPosition = Hardware.slidesPosition;
        }

        public void run() {
            while (opModeIsActive()) {
                if (gamepad1.dpad_up && slides_encoder_target < 2500) {
                    slides_encoder_target += 1;
                }
                else if (gamepad1.dpad_down && slides_encoder_target > 0) {
                    slides_encoder_target -= 1;
                }
                if (BigBird.slides.getCurrentPosition() > slidesPosition.slides_position) {
                    BigBird.slides.setPower(.3);
                }
                else if (BigBird.slides.getCurrentPosition() < slidesPosition.slides_position) {
                    BigBird.slides.setPower(.4);
                }
                else if (BigBird.slides.getCurrentPosition() == slidesPosition.slides_position) {
                    BigBird.slides.setPower(0);
                }
                BigBird.slides.setTargetPosition(slides_encoder_target);
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
            while (opModeIsActive()) {

                if (gamepad1.x) {
                    grabber.claw.setPosition(Math.min(grabber.claw.getPosition()+.01, 1));
                }
                if (gamepad1.y) {
                    grabber.claw.setPosition(Math.max(grabber.claw.getPosition()-.01, 0));
                }
                if (gamepad1.a) {
                    grabber.wrist.setPosition(Math.min(grabber.wrist.getPosition()+.01, 1));
                }
                if (gamepad1.b) {
                    grabber.wrist.setPosition(Math.max(grabber.wrist.getPosition()-.01, 0));
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
}