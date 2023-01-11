package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

    public Servo claw;
    public Servo left_wrist;
    public Servo right_wrist;
    public static double claw_open = 0;
    public static double claw_closed = 0.2;
    public static double upright = 1;
    public static double upside_down = 0.5;
    boolean closed;
    boolean isFlipped;

    // initializes motors & servos
    public Grabber(HardwareMap ahwMap) {
        claw = ahwMap.get(Servo.class, "claw"); // control hub port ?
        left_wrist = ahwMap.get(Servo.class, "left wrist"); // control hub port ?
        right_wrist = ahwMap.get(Servo.class, "right wrist"); // control hub port ?
        claw.setDirection(Servo.Direction.REVERSE);
        closed = false;
    }

    public void closeClaw() {
        claw.setPosition(claw_closed);
        closed = true;
    }
    public void openClaw() {
        claw.setPosition(claw_open);
        closed = false;
    }
    public void flipGrabberFace() {
        left_wrist.setPosition(upside_down);
        right_wrist.setPosition(upside_down);
        isFlipped = true;
    }
    public void rightGrabberFace() {
        left_wrist.setPosition(upright);
        right_wrist.setPosition(upright);
        isFlipped = false;
    }

}
