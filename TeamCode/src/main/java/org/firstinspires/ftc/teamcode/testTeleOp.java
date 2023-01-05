package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//hardware
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

    int downPosition = 0;
    int middlePosition = 300;
    int upPosition = 600;


    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        Drivetrain dt = BigBird.getDrivetrain();

        BigBird.init();
        waitForStart();

        DrivetrainSpeedUpdateThread driveSpeed = new DrivetrainSpeedUpdateThread(dt.FL, dt.BL, dt.FR, dt.BR);
        driveSpeed.start();

        SlidesPosition slidesPosition = SlidesPosition.FRONT_GROUND;
        MoveSlidesThread slidesThread = new MoveSlidesThread(slidesPosition, BigBird, BigBird.slides.getCurrentPosition(), slidesPosition.slides_position, BigBird.elbow1.getPosition(), slidesPosition.elbow_position);
        slidesThread.start();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                slidesPosition = SlidesPosition.FRONT_HIGH;
                //slidesThread = new MoveSlidesThread(slidesPosition, BigBird, BigBird.slides.getCurrentPosition(), slidesPosition.slides_position, BigBird.elbow1.getPosition(), slidesPosition.elbow_position);
                //slidesThread.start();
            }
            else if (gamepad1.dpad_left || gamepad1.dpad_right) {
                slidesPosition = SlidesPosition.FRONT_MIDDLE;
                //slidesThread = new MoveSlidesThread(slidesPosition, BigBird, BigBird.slides.getCurrentPosition(), slidesPosition.slides_position, BigBird.elbow1.getPosition(), slidesPosition.elbow_position);
                //slidesThread.start();
            }
            else if (gamepad1.dpad_down) {
                slidesPosition = (slidesPosition == SlidesPosition.FRONT_LOW) ? SlidesPosition.FRONT_GROUND : SlidesPosition.FRONT_LOW;
                //slidesThread = new MoveSlidesThread(slidesPosition, BigBird, BigBird.slides.getCurrentPosition(), slidesPosition.slides_position, BigBird.elbow1.getPosition(), slidesPosition.elbow_position);
                //slidesThread.start();
            }
            if (BigBird.slides.getCurrentPosition() > slidesPosition.slides_position) {
                BigBird.slides.setPower(.2);
            }
            else if (BigBird.slides.getCurrentPosition() < slidesPosition.slides_position) {
                BigBird.slides.setPower(1);
            }
            else if (BigBird.slides.getCurrentPosition() == slidesPosition.slides_position) {
                BigBird.slides.setPower(0);
            }
            BigBird.slides.setTargetPosition(slidesPosition.slides_position);

            telemetry.addData("Elbow 1 position: ", BigBird.elbow1.getPosition());
            telemetry.addData("Slides: ", BigBird.slides.getCurrentPosition());
            telemetry.addData("Slides position: ", slidesPosition.name);
            telemetry.update();
        }
        slidesThread.interrupt();
    }

    public class DrivetrainSpeedUpdateThread extends Thread {

        DcMotor FL;
        DcMotor BL;
        DcMotor FR;
        DcMotor BR;

        public DrivetrainSpeedUpdateThread(DcMotor m1, DcMotor m2, DcMotor m3, DcMotor m4) {
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

                adjustMotorPower(FL, FLTargetSpeed, speedIncrement);
                adjustMotorPower(BL, BLTargetSpeed, speedIncrement);
                adjustMotorPower(FR, FRTargetSpeed, speedIncrement);
                adjustMotorPower(BR, BRTargetSpeed, speedIncrement);

                try {Thread.sleep(5);} catch (InterruptedException e) {e.printStackTrace();}
            }
        }
    }

    public static class MoveSlidesThread extends Thread {

        SlidesPosition slidesPosition;
        DcMotor slides = null;
        Servo elbow1 = null;
        Servo elbow2 = null;
        int slides_starting_position = 0;
        int slides_target_position = 0;
        double elbow_starting_position = 0;
        double elbow_target_position = 0;

        public MoveSlidesThread(SlidesPosition sp, Hardware BigBird, int ssp, int tsp, double sep, double tep) {
            slidesPosition = sp;
            slides = BigBird.slides;
            elbow1 = BigBird.elbow1;
            elbow2 = BigBird.elbow2;
            slides_starting_position = ssp;
            slides_target_position = tsp;
            elbow_starting_position = sep;
            elbow_target_position = tep;
        }

        @Override
        public void run() {
            slides.setTargetPosition(slides_target_position);
            if (slides_starting_position < slides_target_position) {
                slides.setPower(.4);
                try {Thread.sleep(250);} catch (InterruptedException ignored) {}
                elbow1.setPosition(elbow_target_position);
                elbow2.setPosition(elbow_target_position);
            }
            else if (slides_starting_position > slides_target_position) {
                slides.setPower(0);
                elbow1.setPosition(elbow_target_position);
                elbow2.setPosition(elbow_target_position);
                try {Thread.sleep(250);} catch (InterruptedException ignored) {}
                slides.setPower(.4);
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
}