package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class doCycles extends LinearOpMode {

    // Distance traveled right now is more than one revolution
    // 43.4 cm per 537 ticks
    // 17.087 inches per 537 ticks

    double topConePosition = 0.95; // elbow position to grab top cone
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.slides.setTargetPosition(0);
        BigBird.slides.setPower(0);
        BigBird.grabber.openClaw();
        BigBird.elbowMove(0.95);
        waitForStart();

        BigBird.dt.driveForward(0.2, 0.2);
        BigBird.grabber.closeClaw();

        for (double grabberAngle = topConePosition; grabberAngle <= 1.00; grabberAngle += (1.00-topConePosition) / 5) {
            BigBird.autoDeposit(true,0, SlidesTarget.BACK_HIGH.elbow_position, grabberAngle);
            BigBird.grabber.closeClaw();
            Thread.sleep(500);
        }
    }

}
