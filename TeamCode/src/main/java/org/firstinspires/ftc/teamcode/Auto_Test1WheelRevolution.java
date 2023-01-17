package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class Auto_Test1WheelRevolution extends LinearOpMode {

    // Distance traveled right now is more than one revolution
    // 43.4 cm per 537 ticks
    // 17.087 inches per 537 ticks

    static double ticks_per_revolution = 537.6;
    double power = 0.25;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware BigBird = new Hardware(hardwareMap, telemetry);
        BigBird.init();
        waitForStart();

        BigBird.dt.setMotorTargets((int)(ticks_per_revolution), (int)(ticks_per_revolution), (int)(ticks_per_revolution), (int)(ticks_per_revolution));
        BigBird.dt.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        while (BigBird.dt.FL.getCurrentPosition() < BigBird.dt.FL.getTargetPosition()) {
            BigBird.dt.setMotorPower(power, power, power, power);
        }
        BigBird.dt.setMotorPower(0,0,0,0);
        BigBird.dt.setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
