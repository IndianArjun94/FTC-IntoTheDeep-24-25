package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name = "PID Testing (Forward)", group = "Test")
public class PIDTesting2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);

        waitForStart();

        Commons.PID_forward(12);

        Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.35f;

        Commons.PID_forward(12);
    }
}
