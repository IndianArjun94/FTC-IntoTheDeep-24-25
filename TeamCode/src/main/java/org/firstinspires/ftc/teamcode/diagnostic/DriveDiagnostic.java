package org.firstinspires.ftc.teamcode.diagnostic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Drive Diagnostic", group = "Diagnostic")
public class DriveDiagnostic extends OpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public float leftMotorSpeed = 0;
    public float rightMotorSpeed = 0;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            frontLeft.setPower(1);
        } else {
            frontLeft.setPower(0);
        }

        if (gamepad1.y) {
            frontRight.setPower(1);
        } else {
            frontRight.setPower(0);
        }

        if (gamepad1.b) {
            backRight.setPower(1);
        } else {
            backRight.setPower(0);
        }

        if (gamepad1.a) {
            backLeft.setPower(1);
        } else {
            backLeft.setPower(0);
        }
    }
}
