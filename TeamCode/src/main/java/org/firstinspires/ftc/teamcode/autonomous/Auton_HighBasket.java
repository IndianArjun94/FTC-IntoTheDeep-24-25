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

    public int rightAngleWaitTime = 900;

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

        moveForward(0.5f); // move forward a little bit
        sleep(300);

        rotateLeft(0.5f); // turn left 90 degrees
        sleep(rightAngleWaitTime);

        moveForward(0.5f); // go to the baskets
        sleep(1385);

        rotateLeft(0.5f);
        sleep(rightAngleWaitTime/2);

        moveForward(0.4);
        sleep(102);

//        frontLeftMotor.setPower(0.4f);
//        frontRightMotor.setPower(-0.6f);
//        backLeftMotor.setPower(-0.4f);
//        backRightMotor.setPower(0.6f);

        sleep(75);
        stopMotors();

        armMotor.setPower(0.8);
        sleep(1190);
        int lockingPosition = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(lockingPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4);

        viperSlideMotor.setPower(-0.8);
        while (viperSlideMotor.getCurrentPosition() > -7567) {

        }
        viperSlideMotor.setPower(0);

        intakeServo.setPower(0.5);

        sleep(2500);

        viperSlideMotor.setPower(0.8);
        while (viperSlideMotor.getCurrentPosition() < 0) {

        }
        viperSlideMotor.setPower(0);

        armMotor.setPower(-0.6);
        sleep(1165);
        armMotor.setPower(0);

    }
}
