package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Main extends LinearOpMode {

    public DcMotor motor;

    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor0");

        motor.setPower(0.5);
        sleep(5000);
        motor.setPower(0);
    }
}
