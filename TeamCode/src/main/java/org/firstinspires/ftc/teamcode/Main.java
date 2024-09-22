package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Testing Motors", group = "Learning FTC")
public class Main extends OpMode {

    public DcMotorEx motor;

    public void print(String value, String value1) {
        telemetry.addData(value, value1);
        telemetry.update();
    }

    @Override
    public void init() {
        //        Initialization
        motor = hardwareMap.get(DcMotorEx.class, "motor0");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        print("Robot Status", "Initialized");
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.right_stick_y);
        print("Gamepad 1: ", Float.toString(gamepad1.right_stick_y));
    }
}
