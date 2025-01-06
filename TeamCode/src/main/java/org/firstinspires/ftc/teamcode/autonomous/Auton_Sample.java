package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name = "Auton Samples", group = "Auton FINAL")
public class Auton_Sample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);
        Commons.moveForward(6, 0.5);
        Commons.PID_rotateLeft(90, 0.7);
        Commons.PID_forward(42, 0.8);
        Commons.PID_rotateLeft(45, 0.5);
    }
}
