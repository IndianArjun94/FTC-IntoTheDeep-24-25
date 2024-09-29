package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp Program", group = "FTC Code")
public class Main extends OpMode {

    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.8F;
    public final float ARMROT_SPEED_CAP = 0.3F;

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public DcMotor armMotor;

    public Servo armYawServo;
    public CRServo intakeServo;

    public float leftMotorSpeed = 0;
    public float rightMotorSpeed = 0;

    public float armYawServoPosition = 0F;

    public void print(String value, String value1) {
        telemetry.addData(value, value1);
        telemetry.update();
    }

    public void update() {
//        Robot Movement
        leftMotor.setPower(-leftMotorSpeed*MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        rightMotor.setPower(rightMotorSpeed*MOTOR_MULTIPLIER_PERCENTAGE_CAP);
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
        if (gamepad1.left_stick_y != 0) {
            leftMotorSpeed = gamepad1.left_stick_y;
            rightMotorSpeed = gamepad1.left_stick_y;
        } else {
            leftMotorSpeed = 0;
            rightMotorSpeed = 0;
        }

        if (gamepad1.right_stick_x != 0) {
            leftMotorSpeed -= gamepad1.right_stick_x;
            rightMotorSpeed += gamepad1.right_stick_x;
        }

        update();


//        Robot Arm
        if (gamepad1.y) {
            armMotor.setPower(ARMROT_SPEED_CAP);
        } else if (gamepad1.a) {
            armMotor.setPower(-ARMROT_SPEED_CAP);
        } else {
            armMotor.setPower(0);
        }

//        Arm Yaw Servo & Intake
//        TODO Arm Yaw Servo
        if (gamepad1.left_bumper) {
            if (armYawServoPosition > -1) {
                armYawServoPosition -= 1;
            }
        } else if (gamepad1.right_bumper) {
            if (armYawServoPosition < 1) {
                armYawServoPosition += 1;
            }
        }

    }
}
