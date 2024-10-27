package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Program", group = "FTC Code")
public class TeleOp extends OpMode {
    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.8F;
    public final float ARMROT_SPEED_CAP = 0.5F;

    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor armMotor;

    public Servo armYawServo;
    public CRServo intakeServo;

    public float leftMotorSpeed = 0;
    public float rightMotorSpeed = 0;

    public float armYawServoPosition = 0F;

    public boolean armMotorUnlocked = false;
    public float armIdlePosition = 0;

    public void print(String value, String value1) {
        telemetry.addData(value, value1);
        telemetry.update();
    }

    public void update() {
//        Robot Movement
        leftMotor.setPower(-leftMotorSpeed*MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        rightMotor.setPower(rightMotorSpeed*MOTOR_MULTIPLIER_PERCENTAGE_CAP);

//        Servo Rotation
        armYawServo.setPosition(armYawServoPosition/360);
    }

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armYawServo = hardwareMap.get(Servo.class, "armServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
    }

    @Override
    public void loop() {
//        Robot Movement
        if (gamepad1.left_stick_y != 0) { // Forward/Backward
            leftMotorSpeed = gamepad1.left_stick_y;
            rightMotorSpeed = gamepad1.left_stick_y;
        } else {
            leftMotorSpeed = 0;
            rightMotorSpeed = 0;
        }

        if (gamepad1.right_stick_x != 0) { // Left/Right
            leftMotorSpeed -= gamepad1.right_stick_x;
            rightMotorSpeed += gamepad1.right_stick_x;
        }

//        Robot Arm
        if (gamepad2.y) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(ARMROT_SPEED_CAP);
            armMotorUnlocked = true;
        } else if (gamepad2.a) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(-ARMROT_SPEED_CAP);
            armMotorUnlocked = true;
        } else {
            if (armMotorUnlocked) {
                armMotorUnlocked = false;
                armIdlePosition = armMotor.getCurrentPosition();
                armMotor.setTargetPosition((int)armIdlePosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.15);
            }
        }

//        Arm Yaw Servo
        if (gamepad2.left_bumper) { // Arm Yaw Servo
            if (armYawServoPosition > -120) {
                armYawServoPosition += 1;
            }
        } else if (gamepad2.right_bumper) { // Intake Servo
            if (armYawServoPosition < 120) {
                armYawServoPosition += -1;
            }
        }

//        Intake Servo
        if (gamepad2.right_trigger != 0) {
            intakeServo.setPower(1);
        } else if (gamepad2.left_trigger != 0) {
            intakeServo.setPower(-1);
        }

        update();

    }
}
