package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auton_HighBasket")
public class Auton_HighBasket extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor armMotor;
    public DcMotor viperSlideMotor;

    public CRServo intakeServo;

    public void moveForward(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void rotateLeft(double power) {
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);
    }

    public void rotateRight(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
    }


    public void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlide");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        moveForward(0.5f);
        sleep(250);
        rotateLeft(0.5f);
        sleep(1000);
        moveForward(1);
        sleep(3500);

        frontLeftMotor.setPower(0.4f);
        frontRightMotor.setPower(-0.6f);
        backLeftMotor.setPower(-0.4f);
        backRightMotor.setPower(0.6f);

        sleep(1250);
        stopMotors();

        armMotor.setPower(0.8);
        sleep(1250);
        int lockingPosition = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(lockingPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4);

        viperSlideMotor.setTargetPosition(-8000);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideMotor.setPower(1);

        intakeServo.setPower(0.5);
    }
}
