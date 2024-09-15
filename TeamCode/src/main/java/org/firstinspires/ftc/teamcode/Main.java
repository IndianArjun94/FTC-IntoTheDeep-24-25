package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Testing Motors", group = "Learning FTC")
public class Main extends LinearOpMode {

    public DcMotorEx motor;

    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor0");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setTargetPosition(5 * (int) 537.6);
        motor.setPower(0.1);

        while (motor.isBusy()) {
            telemetry.addData("Motor Status", "Running...");
        }
    }
}
