package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Program", group = "Final TeleOp")
public class TeleOp extends OpMode {
    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.8F;
    public final float ARMROT_SPEED_CAP = 0.5F;

    public final int VIPER_SLIDE_MIN = 0;
    public final int VIPER_SLIDE_MAX = 3000;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor armMotor;
    public DcMotor viperSlide;

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
        frontLeft.setPower(-frontLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        frontRight.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backLeft.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backRight.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        viperSlide.setPower(viperSlideSpeed);
    }

    public void setMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void changeMotorPower(double power) {
        frontLeft.setPower(frontLeftMotorSpeed + power);
        frontRight.setPower(frontRightMotorSpeed + power);
        backLeft.setPower(backLeftMotorSpeed + power);
        backRight.setPower(backRightMotorSpeed + power);
    }

    public void changeMotorPowerLateral(double power) {
        frontLeft.setPower(frontLeftMotorSpeed - power);
        frontRight.setPower(frontRightMotorSpeed + power);
        backLeft.setPower(backLeftMotorSpeed + power);
        backRight.setPower(backRightMotorSpeed - power);
    }

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
//        Robot Movement
        if (gamepad1.left_stick_y != 0) { // Forward/Backward
            setMotorPower(gamepad1.left_stick_y);
        } else {
            setMotorPower(0);
        }

        if (gamepad1.left_stick_x != 0) {
            changeMotorPowerLateral(gamepad1.left_stick_x);
        }

        if (gamepad1.right_stick_x != 0) { // Left/Right
            frontLeftMotorSpeed -= gamepad1.right_stick_x;
            frontRightMotorSpeed += gamepad1.right_stick_x;
        }

//        Robot Arm
        if (gamepad1.y || gamepad2.y) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(ARMROT_SPEED_CAP);
//            armMotorUnlocked = true;
        } else if (gamepad1.a || gamepad2.a) {
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
            viperSlide.setTargetPosition(VIPER_SLIDE_MAX);
            viperSlide.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger != 0) {
            viperSlide.setTargetPosition(VIPER_SLIDE_MIN);
            viperSlide.setPower(-gamepad1.left_trigger);
        } else {
            viperSlide.setPower(0);
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
