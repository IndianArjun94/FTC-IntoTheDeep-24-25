package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp GOLDEN", group = "TeleOp")
public class TeleOp_GOLDEN extends OpMode {
    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.5F;
    public final float ARMROT_SPEED_CAP = 0.7F;

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor armMotor;
    public DcMotor viperSlideMotor;

    public CRServo intakeServo;

    public Servo clawArm;

    public Servo claw;

    public float frontLeftMotorSpeed = 0;
    public float frontRightMotorSpeed = 0;
    public float backLeftMotorSpeed = 0;
    public float backRightMotorSpeed = 0;

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
        clawArm = hardwareMap.get(Servo.class, "clawArm");
        claw = hardwareMap.get(Servo.class, "claw");

        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.setMsTransmissionInterval(180);

    }

    @Override
    public void loop() {

        telemetry.addData("Viper Slide Position: ", viperSlideMotor.getCurrentPosition());
        telemetry.addData("Arm Motor Position: ", armMotor.getCurrentPosition());
        telemetry.addData("Front Left Motor Speed: ", frontLeftMotorSpeed);
        telemetry.addData("Front Right Motor Speed:", frontRightMotorSpeed);
        telemetry.addData("Back Left Motor Speed: ", backLeftMotorSpeed);
        telemetry.addData("Back Right Motor Speed: ", backRightMotorSpeed);
        telemetry.addData("Gamepad 1 Button: ", gamepad1);
        telemetry.addData("Claw Arm Position", clawArm.getPosition());
        telemetry.addData("Claw Position", claw.getPosition());

        telemetry.addLine();

        telemetry.addData("Viper Slide Max: -2976", "");


        frontLeftMotorSpeed = 0;
        frontRightMotorSpeed = 0;
        backLeftMotorSpeed = 0;
        backRightMotorSpeed = 0;

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

        //automatic hang
        if (gamepad1.ps && gamepad2.ps) {
            frontLeftMotor.setPower(0.5);
            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(-0.75);

            telemetry.addData("hanging", "");

            //sleep for 1500 (try/catch)
            try {
                sleep(2000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            armMotorUnlocked = false;
            armIdlePosition = armMotor.getCurrentPosition();
            armMotor.setTargetPosition((int)armIdlePosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.4);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            intakeServo.setPower(0);

            telemetry.addData("hanged","");

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

//        Viper Slide
        if (gamepad2.right_trigger != 0) { // Extend
            viperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            viperSlideMotor.setPower(-gamepad2.right_trigger);
        } else if (gamepad2.left_trigger != 0) { // Retract
            viperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            viperSlideMotor.setPower(gamepad2.left_trigger);
        } else {
            viperSlideMotor.setTargetPosition(viperSlideMotor.getCurrentPosition());
            viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperSlideMotor.setPower(0.4);
        }

//        Intake Servo
        if (gamepad2.right_bumper) {
            intakeServo.setPower(1);
        } else if (gamepad2.left_bumper) {
            intakeServo.setPower(-1);
        } else if (gamepad2.back) {
            intakeServo.setPower(0);
        }

//       Stuck on bar
        if (gamepad1.dpad_down) {
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
            if (gamepad1.dpad_down) {
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }
        }

//        Viper Limit
        if (viperSlideMotor.getCurrentPosition() < -2976) {
            viperSlideMotor.setPower(0);
            while (viperSlideMotor.getCurrentPosition() < -2976) {
                viperSlideMotor.setPower(0.5);
            }
        }
//          Claw arm
        while (gamepad2.y) {
            clawArm.setPosition(clawArm.getPosition()+2);
        }
        while (gamepad2.a) {
            clawArm.setPosition(clawArm.getPosition()-2);
        }
//          Claw
        while (gamepad2.b) {
            claw.setPosition(90);
        }
        while (gamepad2.x) {
            claw.setPosition(0);
        }
//          Claw Arm Limit
        if (clawArm.getPosition() >= 300) {
            clawArm.setPosition(299);
        }
        if (clawArm.getPosition() <= 0 ){
            clawArm.setPosition(1);
        }
        telemetry.update();
        update();
    }
}