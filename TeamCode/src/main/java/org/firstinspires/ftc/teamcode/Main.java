package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TeleOp Program", group = "FTC Code")
public class Main extends OpMode {

    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;

    public float leftMotorSpeed = 0;
    public float rightMotorSpeed = 0;

    public void print(String value, String value1) {
        telemetry.addData(value, value1);
        telemetry.update();
    }

    public void update() {
        leftMotor.setPower(leftMotorSpeed);
        rightMotor.setPower(rightMotorSpeed);
    }

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        TODO - reverse the motor that needs to be reversed

        print("Robot Status", "Initialized");
    }

    @Override
    public void loop() {
        leftMotorSpeed += gamepad1.left_stick_y + gamepad1.left_stick_x;
        rightMotorSpeed += gamepad1.left_stick_y - gamepad1.left_stick_x;
        update();
    }
}
