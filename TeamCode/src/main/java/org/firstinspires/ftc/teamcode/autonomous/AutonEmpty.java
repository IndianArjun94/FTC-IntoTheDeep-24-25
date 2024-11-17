package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Empty Auton", group = "Auton Final")
public class AutonEmpty extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        sleep(35000);
    }
}
