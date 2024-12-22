package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name = "PID Testing", group = "Test")
public class PIDTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap);

        Commons.PID_rotate(90, this::opModeIsActive);
    }
}
