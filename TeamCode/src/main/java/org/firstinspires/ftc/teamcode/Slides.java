package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slides {
    public DcMotor slides;
    public Servo servo1, servo2;

    // initializes motors & servos
    public Slides(HardwareMap ahwMap) {
        slides = ahwMap.get(DcMotor.class, "slides"); // control hub port ?
        servo1 = ahwMap.get(Servo.class, "slides1"); // control hub port ?
        servo2 = ahwMap.get(Servo.class, "slides2"); // control hub port ?
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}