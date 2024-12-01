package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Alpha", group = "TeleOp")
public class TeleOp_ALPHA extends OpMode {
    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.8F;
    public final float ARMROT_SPEED_CAP = 0.5F;

    public final int VIPER_SLIDE_MIN = 0;
    public final int VIPER_SLIDE_MAX = 1000;

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor armMotor;
    public DcMotor viperSlideMotor;

    public CRServo intakeServo;

    public float frontLeftMotorSpeed = 0;
    public float frontRightMotorSpeed = 0;
    public float backLeftMotorSpeed = 0;
    public float backRightMotorSpeed = 0;
    public float viperSlideSpeed = 0;

//    Old Arm-Locking Mechanism
    public boolean armMotorUnlocked = false;
    public float armIdlePosition = 0;

    public void print(String value, String value1) {
        telemetry.addData(value, value1);
        telemetry.update();
    }

    public void update() {
//        Robot Movement
        frontLeftMotor.setPower(frontLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        frontRightMotor.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backLeftMotor.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backRightMotor.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        viperSlideMotor.setPower(viperSlideSpeed);
    }

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlide");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
//        Robot Movement
        if (gamepad1.left_stick_y != 0) { // Forward/Backward
            frontLeftMotorSpeed = gamepad1.left_stick_y;
            frontRightMotorSpeed = gamepad1.left_stick_y;
            backLeftMotorSpeed = gamepad1.left_stick_y;
            backRightMotorSpeed = gamepad1.left_stick_y;
        } else {
            frontLeftMotorSpeed = 0;
            frontRightMotorSpeed = 0;
            backLeftMotorSpeed = 0;
            backRightMotorSpeed = 0;
        }
        if (gamepad1.left_stick_x != 0) {
            frontLeftMotorSpeed -= gamepad1.left_stick_x;
            frontRightMotorSpeed += gamepad1.left_stick_x;
            backLeftMotorSpeed += gamepad1.left_stick_x;
            backRightMotorSpeed -= gamepad1.left_stick_x;
        }
        if (gamepad1.right_stick_x != 0) { // Left/Right
            frontLeftMotorSpeed -= gamepad1.right_stick_x;
            backLeftMotorSpeed -= gamepad1.right_stick_x;
            frontRightMotorSpeed += gamepad1.right_stick_x;
            backRightMotorSpeed += gamepad1.right_stick_x;
        }
//        Robot Arm
        if (gamepad2.y) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(ARMROT_SPEED_CAP);
//            armMotorUnlocked = true;
        } else if (gamepad2.a) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(-ARMROT_SPEED_CAP);
//            armMotorUnlocked = true;
        } else {
//            if (armMotorUnlocked) {
//                armMotorUnlocked = false;
//                armIdlePosition = armMotor.getCurrentPosition();
//                armMotor.setTargetPosition((int)armIdlePosition);
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armMotor.setPower(0.15);
//            }
            armMotor.setPower(0);
        }
//        Arm Lock Servo - DISABLED
//        if (gamepad1.x) {
//            hangServo.setPosition(hangServoBaseValue-180);
//        } else if (gamepad1.b) {
//            hangServo.setPosition(hangServoBaseValue);
//        }

//        Arm Yaw Servo - DISABLED
//        if (gamepad2.left_bumper) { // Arm Yaw Servo
//            if (armYawServoPosition < 300) {
//                armYawServoPosition += 1;
//            }
//        } else if (gamepad2.right_bumper) { // Intake Servo
//            if (armYawServoPosition > -300) {
//                armYawServoPosition += -1;
//            }
//        }
//        Viper Slide
        if (gamepad1.right_trigger != 0) {
            viperSlideMotor.setPower(1);
        } else if (gamepad1.left_trigger != 0) {
            viperSlideMotor.setPower(-1);
        } else {
            viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            viperSlideMotor.setPower(0);
        }
//        Intake Servo
        if (gamepad2.right_bumper) {
            intakeServo.setPower(1);
        } else if (gamepad2.left_bumper) {
            intakeServo.setPower(-1);
        } else if (gamepad2.x) {
            intakeServo.setPower(0);
        }


        update();
    }
}