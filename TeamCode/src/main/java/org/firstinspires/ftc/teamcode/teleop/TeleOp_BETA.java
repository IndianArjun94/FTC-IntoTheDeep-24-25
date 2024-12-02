package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp BETA", group = "TeleOp")
public class TeleOp_BETA extends OpMode {
    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.5F;
    public final float ARMROT_SPEED_CAP = 0.7F;

    public final int VIPER_SLIDE_MIN = -12000;
    public final int VIPER_SLIDE_MAX = 2000;

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
        backLeftMotor.setPower(backLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backRightMotor.setPower(backRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);

        viperSlideMotor.setPower(-viperSlideSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
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

        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        frontLeftMotorSpeed = 0;
        frontRightMotorSpeed = 0;
        backLeftMotorSpeed = 0;
        backRightMotorSpeed = 0;

        viperSlideSpeed = 0;

        float left_stick_x = gamepad1.left_stick_x;
        float left_stick_y = gamepad1.left_stick_y;
        float right_stick_x = gamepad1.right_stick_x;

//        Forward/Backward Movement
        if (left_stick_y != 0) {
            frontLeftMotorSpeed = -left_stick_y;
            frontRightMotorSpeed = -left_stick_y;
            backLeftMotorSpeed = -left_stick_y;
            backRightMotorSpeed = -left_stick_y;
        }

//        Lateral Movement
        if (left_stick_x != 0) {
            frontLeftMotorSpeed += left_stick_x;
            frontRightMotorSpeed -= left_stick_x;
            backLeftMotorSpeed -= left_stick_x;
            backRightMotorSpeed += left_stick_x;
        }

//        Rotation
        if (right_stick_x != 0) {
            frontLeftMotorSpeed += right_stick_x;
            backLeftMotorSpeed += right_stick_x;
            frontRightMotorSpeed -= right_stick_x;
            backRightMotorSpeed -= right_stick_x;
        }

//        Robot Arm
        if (gamepad2.dpad_up) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(ARMROT_SPEED_CAP);
            armMotorUnlocked = true;
        } else if (gamepad2.dpad_down) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(-ARMROT_SPEED_CAP);
            armMotorUnlocked = true;
        } else {
            if (armMotorUnlocked) {
                armMotorUnlocked = false;
                armIdlePosition = armMotor.getCurrentPosition();
                armMotor.setTargetPosition((int)armIdlePosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.4);
            }
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
        if (gamepad2.left_trigger != 0 && viperSlideMotor.getCurrentPosition() >= VIPER_SLIDE_MIN) {
            viperSlideSpeed -= gamepad2.right_trigger;
        } else if (gamepad2.right_trigger != 0 && viperSlideMotor.getCurrentPosition() <= VIPER_SLIDE_MAX) {
            viperSlideSpeed += gamepad2.left_trigger;
        } else {
            viperSlideMotor.setPower(0);
        }

        print("ViperSlide Position", Float.toString(viperSlideMotor.getCurrentPosition()));

//        Intake Servo
        if (gamepad2.right_bumper) {
            intakeServo.setPower(1);
        } else if (gamepad2.left_bumper) {
            intakeServo.setPower(-1);
        } else if (gamepad2.right_bumper && gamepad2.left_bumper) {
            intakeServo.setPower(0);
        }


        update();
    }
}
