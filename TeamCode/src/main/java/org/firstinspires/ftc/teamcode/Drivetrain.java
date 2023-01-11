package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class Drivetrain {
    //instantiates motor names for drivetrain
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;
    static int ticks_per_tile = 1000; // constant for strafe methods

    // initializes motors
    public Drivetrain(HardwareMap ahwMap){
        FL = ahwMap.get(DcMotor.class, "FL"); // control hub port 2
        FR = ahwMap.get(DcMotor.class, "FR"); // control hub port 3
        BL = ahwMap.get(DcMotor.class, "BL"); // control hub port 0
        BR = ahwMap.get(DcMotor.class,"BR");  // control hub port 1

        setMotorPower(0,0,0,0);

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);


        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setMotorPower(double fl, double fr, double bl, double br){
        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
    }

    public void setMotorTargets(int fl, int fr, int bl, int br){
        FL.setTargetPosition(fl);
        FR.setTargetPosition(fr);
        BL.setTargetPosition(bl);
        BR.setTargetPosition(br);
    }

    //sets power for motors based on normalized powers
    public void setMotorPower(double[]powers){
        FL.setPower(powers[0]);
        FR.setPower(powers[1]);
        BL.setPower(powers[2]);
        BR.setPower(powers[3]);
    }
    public double[] normalizePowers(double[] powers){
        Arrays.sort(powers);
        if(powers[3] > 1){
            powers[0] /= powers[3];
            powers[1] /= powers[3];
            powers[2] /= powers[3];
            powers[3] /= powers[3];
        }
        return powers;
    }

    public void strafeLeft(double num_tiles, double power) {
        setMotorTargets(FL.getTargetPosition() - (int)((num_tiles * ticks_per_tile)),
                        FR.getTargetPosition() + (int)((num_tiles * ticks_per_tile)),
                        BL.getTargetPosition() + (int)((num_tiles * ticks_per_tile)),
                        BR.getTargetPosition() - (int)((num_tiles * ticks_per_tile)));
        setMotorPower(power, power, power, power);
    }
    public void strafeRight(double num_tiles, double power) {
        setMotorTargets(FL.getTargetPosition() + (int)((num_tiles * ticks_per_tile)),
                        FR.getTargetPosition() - (int)((num_tiles * ticks_per_tile)),
                        BL.getTargetPosition() - (int)((num_tiles * ticks_per_tile)),
                        BR.getTargetPosition() + (int)((num_tiles * ticks_per_tile)));
        setMotorPower(power, power, power, power);
    }
    public void driveForward(double num_tiles, double power) {
        setMotorTargets(FL.getTargetPosition() + (int)((num_tiles * ticks_per_tile)),
                        FR.getTargetPosition() + (int)((num_tiles * ticks_per_tile)),
                        BL.getTargetPosition() + (int)((num_tiles * ticks_per_tile)),
                        BR.getTargetPosition() + (int)((num_tiles * ticks_per_tile)));
        setMotorPower(power, power, power, power);
    }
    public void driveBackward(double num_tiles, double power) {
        setMotorTargets(FL.getTargetPosition() - (int)((num_tiles * ticks_per_tile)),
                FR.getTargetPosition() - (int)((num_tiles * ticks_per_tile)),
                BL.getTargetPosition() - (int)((num_tiles * ticks_per_tile)),
                BR.getTargetPosition() - (int)((num_tiles * ticks_per_tile)));
        setMotorPower(power, power, power, power);
    }
}
