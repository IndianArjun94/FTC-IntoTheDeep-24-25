package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Testing Motors", group = "Learning FTC")
public class Main extends LinearOpMode {

    public DcMotorEx motor;

    public void runOpMode() throws InterruptedException {
        waitForStart();

        motor = hardwareMap.get(DcMotorEx.class, "motor0");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setTargetPosition(50 * (int) 537.6);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        print("Motor Status", "Initialized");

        motor.setPower(1);

        while (motor.isBusy()) {
            print("Motor Status", "Running...");
        }

        motor.setPower(0);

        print("Done", "Done");
    }

    public void print(String value, String value1) {
        telemetry.addData(value, value1);
        telemetry.update();
    }
}
