package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TeleOp Program", group = "FTC Code")
public class Main extends OpMode {

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public float leftMotorSpeed = 0;
    public float rightMotorSpeed = 0;

    public void print(String value, String value1) {
        telemetry.addData(value, value1);
        telemetry.update();
    }

    public void update() {
        leftMotor.setPower(leftMotorSpeed/2);
        rightMotor.setPower(-rightMotorSpeed/2);
    }

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        TODO - reverse the motor that needs to be reversed

        print("Robot Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y != 0) {
            leftMotorSpeed = gamepad1.left_stick_y;
            rightMotorSpeed = gamepad1.left_stick_y;
        } else {
            leftMotorSpeed = 0;
            rightMotorSpeed = 0;
        }

        if (gamepad1.right_stick_x != 0) {
            leftMotorSpeed += gamepad1.right_stick_x;
            rightMotorSpeed -= gamepad1.right_stick_x;
        }

        update();
    }
}
