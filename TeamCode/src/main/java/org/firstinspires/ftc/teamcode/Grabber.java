package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

    public Servo claw;
    public Servo wrist;
    public static double claw_open = 0.5;
    public static double claw_closed = 0;
    public static double upright = 0;
    public static double upside_down = 1;
    boolean isClosed;
    boolean isFlipped;

    // initializes motors & servos
    public Grabber(HardwareMap ahwMap) {
        claw = ahwMap.get(Servo.class, "claw"); // control hub port ?
        wrist = ahwMap.get(Servo.class, "wrist"); // control hub port ?
        claw.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        isClosed = false;
    }

    public void closeClaw() {
        claw.setPosition(claw_closed);
        isClosed = true;
    }
    public void openClaw() {
        claw.setPosition(claw_open);
        isClosed = false;
    }
    public void flipGrabberFace() {
        wrist.setPosition(upside_down);
        isFlipped = true;
    }
    public void rightGrabberFace() {
        wrist.setPosition(upright);
        isFlipped = false;
    }

}
