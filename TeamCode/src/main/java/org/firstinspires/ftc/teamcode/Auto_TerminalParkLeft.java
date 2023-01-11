package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Auto_TerminalParkLeft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();

        BigBird.dt.strafeLeft(1, .5);
    }

    public void drive(Hardware BigBird, double drive_speed, int duration) throws InterruptedException {
        BigBird.dt.FL.setPower(drive_speed);
        BigBird.dt.FR.setPower(drive_speed);
        BigBird.dt.BL.setPower(drive_speed);
        BigBird.dt.BR.setPower(drive_speed);
        Thread.sleep(duration);
        BigBird.dt.FL.setPower(0);
        BigBird.dt.FR.setPower(0);
        BigBird.dt.BL.setPower(0);
        BigBird.dt.BR.setPower(0);
    }

    public void strafe_left(Hardware BigBird,double drive_speed, int duration) throws InterruptedException {
        BigBird.dt.FL.setPower(-drive_speed);
        BigBird.dt.FR.setPower(drive_speed);
        BigBird.dt.BL.setPower(drive_speed);
        BigBird.dt.BR.setPower(-drive_speed);
        Thread.sleep(duration);
        BigBird.dt.FL.setPower(0);
        BigBird.dt.FR.setPower(0);
        BigBird.dt.BL.setPower(0);
        BigBird.dt.BR.setPower(0);
    }

}
