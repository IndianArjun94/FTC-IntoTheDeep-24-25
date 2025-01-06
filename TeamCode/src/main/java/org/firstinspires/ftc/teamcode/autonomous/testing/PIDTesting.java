package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name = "PID Testing (Turn)", group = "Test")
public class PIDTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);

        waitForStart();

        Commons.PID_forward(24, 0.7);
        Commons.lateralRight(36, 0.6);
        Commons.PID_backward(24, 0.3);
        Commons.lateralLeft(36, 0.5);
        Commons.PID_rotateRight(90, 0.5);
        Commons.PID_rotateLeft(90, 1);
    }
}
