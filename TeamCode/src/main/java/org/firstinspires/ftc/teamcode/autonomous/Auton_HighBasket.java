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

    public int rightAngleWaitTime = 720;

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
        sleep(125);

        rotateLeft(0.5f); // turn left 90 degrees
        sleep(rightAngleWaitTime);

        moveForward(0.5f); // go to the baskets
        sleep(1385);

        rotateLeft(0.5f); // rotate to face the baskets
        sleep(295);

        moveForward(-0.5f); // back up a little bit
        sleep(120);

        stopMotors();

        armMotor.setPower(0.8);
        sleep(1100);
        int lockingPosition = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(lockingPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4);

        viperSlideMotor.setPower(-1);
        while (viperSlideMotor.getCurrentPosition() > -7500) {

        }
        viperSlideMotor.setPower(0); // viper slide up to high basket

        intakeServo.setPower(0.5);

        sleep(750); // put out sample

        intakeServo.setPower(0);

        viperSlideMotor.setPower(1);
        while (viperSlideMotor.getCurrentPosition() < 0) {

        }
        viperSlideMotor.setPower(0); // viper slide down

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(-0.8); // arm down
        sleep(1000);
        armMotor.setPower(0);

        rotateRight(0.5f); // rotate to face floor sample
        sleep(500);

        moveForward(-0.5); // move back to floor sample
        sleep(350);

        frontLeftMotor.setPower(0.5f); // move laterally to the floor sample
        backLeftMotor.setPower(-0.5f);
        frontRightMotor.setPower(-0.5f);
        backRightMotor.setPower(0.5f);
        sleep(675);
        stopMotors();

        armMotor.setPower(0.5f); // move the arm up a little bit
        sleep(400);
        lockingPosition = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(lockingPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4);

        viperSlideMotor.setPower(-0.5); // extend viper slide a little bit
        while (viperSlideMotor.getCurrentPosition() > -650) {

        }
        viperSlideMotor.setPower(0);

        armMotor.setPower(0); // arm down
        sleep(1000);

        intakeServo.setPower(-0.5f); // take in floor sample #1
        sleep(2000);

        sleep(1000000);
    }
}
