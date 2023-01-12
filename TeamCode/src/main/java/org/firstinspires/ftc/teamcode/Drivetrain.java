package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODERS;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class Drivetrain {
    //instantiates motor names for drivetrain
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;
    static int ticks_per_tile = 580; // constant for strafe methods
    static double strafe_constant = 1.3;

    Telemetry telemetry;
    // initializes motors
    public Drivetrain(HardwareMap ahwMap, Telemetry telemetry) {
        FL = ahwMap.get(DcMotor.class, "FL"); // control hub port 2
        FR = ahwMap.get(DcMotor.class, "FR"); // control hub port 3
        BL = ahwMap.get(DcMotor.class, "BL"); // control hub port 0
        BR = ahwMap.get(DcMotor.class, "BR");  // control hub port 1

        setMotorPower(0, 0, 0, 0);

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);


        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
    }

    public void setMotorPower(double fl, double fr, double bl, double br) {
        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
    }

    public void setMotorTargets(int fl, int fr, int bl, int br) {
        FL.setTargetPosition(fl);
        FR.setTargetPosition(fr);
        BL.setTargetPosition(bl);
        BR.setTargetPosition(br);
    }

    //sets power for motors based on normalized powers
    public void setMotorPower(double[] powers) {
        FL.setPower(powers[0]);
        BL.setPower(powers[1]);
        FR.setPower(powers[2]);
        BR.setPower(powers[3]);
    }

    public double[] normalizePowers(double[] powers) {
        Arrays.sort(powers);
        if (powers[3] > 1) {
            powers[0] /= powers[3];
            powers[1] /= powers[3];
            powers[2] /= powers[3];
            powers[3] /= powers[3];
        }
        return powers;
    }

    public void strafeLeft(double num_tiles, double power) throws InterruptedException {
        setMotorModes(STOP_AND_RESET_ENCODER);
        setMotorTargets((int) (-num_tiles * ticks_per_tile * strafe_constant),
                        (int) (num_tiles  * ticks_per_tile * strafe_constant),
                        (int) (num_tiles  * ticks_per_tile * strafe_constant),
                        (int) (-num_tiles * ticks_per_tile * strafe_constant));
        setMotorModes(RUN_TO_POSITION);

        setMotorPower(power, power, power, power);
        while (FL.getCurrentPosition() > FL.getTargetPosition()) {
            telemetry.addData("FL target: ", FL.getTargetPosition());
            telemetry.addData("BL target: ", BL.getTargetPosition());
            telemetry.addData("FR target: ", FR.getTargetPosition());
            telemetry.addData("BR target: ", BR.getTargetPosition());
            telemetry.addData("FL position: ", FL.getCurrentPosition());
            telemetry.addData("BL position: ", BL.getCurrentPosition());
            telemetry.addData("FR position: ", FR.getCurrentPosition());
            telemetry.addData("BR position: ", BR.getCurrentPosition());
            telemetry.update();

            Thread.sleep(10);
        }
        setMotorModes(RUN_WITHOUT_ENCODER);
        setMotorPower(0, 0, 0, 0);


        //drive(180, num_tiles, power);
    }

    public void strafeRight(double num_tiles, double power) throws InterruptedException {
        setMotorModes(STOP_AND_RESET_ENCODER);
        setMotorTargets((int) (num_tiles  * ticks_per_tile * strafe_constant),
                        (int) (-num_tiles * ticks_per_tile * strafe_constant),
                        (int) (-num_tiles * ticks_per_tile * strafe_constant),
                        (int) (num_tiles  * ticks_per_tile * strafe_constant));
        setMotorModes(RUN_TO_POSITION);
        setMotorPower(power, power, power, power);
        while (FL.getCurrentPosition() < FL.getTargetPosition()) {
            Thread.sleep(10);
        }
        setMotorModes(RUN_WITHOUT_ENCODER);
        setMotorPower(0, 0, 0, 0);


        //drive(0, num_tiles, power);
    }

    public void driveForward(double num_tiles, double power) throws InterruptedException {
        setMotorModes(STOP_AND_RESET_ENCODER);
        setMotorTargets((int) (num_tiles * ticks_per_tile),
                        (int) (num_tiles * ticks_per_tile),
                        (int) (num_tiles * ticks_per_tile),
                        (int) (num_tiles * ticks_per_tile));
        setMotorModes(RUN_TO_POSITION);
        setMotorPower(power, power, power, power);
        while (FL.getCurrentPosition() < FL.getTargetPosition()) {
            telemetry.addData("FL target: ", FL.getTargetPosition());
            telemetry.addData("BL target: ", BL.getTargetPosition());
            telemetry.addData("FR target: ", FR.getTargetPosition());
            telemetry.addData("BR target: ", BR.getTargetPosition());
            telemetry.addData("FL position: ", FL.getCurrentPosition());
            telemetry.addData("BL position: ", BL.getCurrentPosition());
            telemetry.addData("FR position: ", FR.getCurrentPosition());
            telemetry.addData("BR position: ", BR.getCurrentPosition());

            telemetry.update();
            Thread.sleep(10);
        }
        setMotorModes(RUN_WITHOUT_ENCODER);
        setMotorPower(0, 0, 0, 0);


        //drive(90, num_tiles, power);
    }

    public void driveBackward(double num_tiles, double power) throws InterruptedException {
        setMotorModes(STOP_AND_RESET_ENCODER);
        setMotorTargets((int) (-num_tiles * ticks_per_tile),
                        (int) (-num_tiles * ticks_per_tile),
                        (int) (-num_tiles * ticks_per_tile),
                        (int) (-num_tiles * ticks_per_tile));
        setMotorModes(RUN_TO_POSITION);
        setMotorPower(power, power, power, power);
        while (FL.getCurrentPosition() > FL.getTargetPosition()) {
            Thread.sleep(10);
        }
        setMotorModes(RUN_WITHOUT_ENCODER);
        setMotorPower(0, 0, 0, 0);


        //drive(270, num_tiles, power);
    }

    public void setMotorModes(DcMotor.RunMode mode) {
        FL.setMode(mode);
        BL.setMode(mode);
        FR.setMode(mode);
        BR.setMode(mode);
    }

    // DOES NOT WORK PROPERLY
    public void drive(double angle, double num_tiles, double power) throws InterruptedException {
        // FL and BR are y+x
        // FR and BL are y-x
        double x = Math.cos(Math.toRadians(angle));
        double y = Math.sin(Math.toRadians(angle));
        setMotorModes(RUN_USING_ENCODER);
        setMotorTargets(FL.getTargetPosition() + (int)(num_tiles * y+x),
                        FR.getTargetPosition() + (int)(num_tiles * y-x),
                        BL.getTargetPosition() + (int)(num_tiles * y-x),
                        BR.getTargetPosition() + (int)(num_tiles * y+x));
        setMotorModes(RUN_TO_POSITION);
        setMotorPower(Math.abs(y+x), Math.abs(y-x), Math.abs(y-x), Math.abs(y+x));
        while (FL.getCurrentPosition() != FL.getTargetPosition() && BL.getCurrentPosition() != BL.getTargetPosition() && FR.getCurrentPosition() != FR.getTargetPosition() && BR.getCurrentPosition() != BR.getTargetPosition()) {
            Thread.sleep(10);
        }
        setMotorPower(0,0,0,0);
        setMotorModes(RUN_WITHOUT_ENCODER);
    }

}