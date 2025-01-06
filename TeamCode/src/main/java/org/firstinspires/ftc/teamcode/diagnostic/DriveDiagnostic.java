package org.firstinspires.ftc.teamcode.diagnostic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
            frontLeft.setPower(0.5);
        } else {
            frontLeft.setPower(0);
        }

        if (gamepad1.y) {
            frontRight.setPower(0.5);
        } else {
            frontRight.setPower(0);
        }

        if (gamepad1.b) {
            backLeft.setPower(0.5);
        } else {
            backLeft.setPower(0);
        }

        if (gamepad1.a) {
            backRight.setPower(0.5);
        } else {
            backRight.setPower(0);
        }
    }
}
