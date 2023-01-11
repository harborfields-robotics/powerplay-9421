package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


// unused as of 1/9/2023

public class Slides {
    public DcMotor slides;
    public Servo servo1, servo2;
    public static int bottom = 0;
    public static int middle = 300;
    public static int top = 600;

    public static double bottom_position = 0.0;
    public static double straight_out = 0.25;
    public static double reaching_up = 0.4;

    // initializes motors & servos
    public Slides(HardwareMap ahwMap) {
        slides = ahwMap.get(DcMotor.class, "slides"); // control hub port ?
        servo1 = ahwMap.get(Servo.class, "elbow 1"); // control hub port ?
        servo2 = ahwMap.get(Servo.class, "elbow 2"); // control hub port ?
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}