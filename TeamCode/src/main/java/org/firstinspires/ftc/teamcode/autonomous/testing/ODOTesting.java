package org.firstinspires.ftc.teamcode.autonomous.testing;

import static org.firstinspires.ftc.teamcode.Commons.PID_backward;
import static org.firstinspires.ftc.teamcode.Commons.PID_forward;
import static org.firstinspires.ftc.teamcode.Commons.PID_goto;
import static org.firstinspires.ftc.teamcode.Commons.PID_rotateLeft;
import static org.firstinspires.ftc.teamcode.Commons.moveForward;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name = "Odometry Test", group = "Test")
public class ODOTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);
        telemetry.setMsTransmissionInterval(250);
        waitForStart();
        PID_goto(10, 10, true, 0.25);
    }
}
