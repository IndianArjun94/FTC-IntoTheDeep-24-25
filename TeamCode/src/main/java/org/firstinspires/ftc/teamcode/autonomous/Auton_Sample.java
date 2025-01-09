package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Commons.PID_forward;
import static org.firstinspires.ftc.teamcode.Commons.PID_rotateLeft;
import static org.firstinspires.ftc.teamcode.Commons.PID_rotateRight;
import static org.firstinspires.ftc.teamcode.Commons.armMotor;
import static org.firstinspires.ftc.teamcode.Commons.intakeServo;
import static org.firstinspires.ftc.teamcode.Commons.lateralLeft;
import static org.firstinspires.ftc.teamcode.Commons.lateralRight;
import static org.firstinspires.ftc.teamcode.Commons.moveBackward;
import static org.firstinspires.ftc.teamcode.Commons.moveForward;
import static org.firstinspires.ftc.teamcode.Commons.print;
import static org.firstinspires.ftc.teamcode.Commons.viperSlideMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name = "Auton SAMPLES", group = "Auton FINAL")
public class Auton_Sample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);
        telemetry.setMsTransmissionInterval(100);
        waitForStart();
        moveForward(12, 0.8);
        PID_rotateLeft(90, 0.8);
        moveForward(32, 0.8f);
        PID_rotateLeft(45, 0.8);
        PID_forward(10, 0.8);

//        Arm Up
        armMotor.setPower(0.8);
        armMotor.setTargetPosition(1980);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() < 1980 && opModeIsActive()) {
            sleep(5);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setPower(0.4);

//        Viper Up
        viperSlideMotor.setPower(-0.75);
        viperSlideMotor.setTargetPosition(-2600);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition()>-2600 && opModeIsActive()) {
            sleep(5);
            print("Viper Slide Position", Integer.toString(viperSlideMotor.getCurrentPosition()));
        }
        intakeServo.setPower(0.5);
        sleep(1500);
        intakeServo.setPower(0);

//        Viper Down
        viperSlideMotor.setTargetPosition(20);
        viperSlideMotor.setPower(1);
        while (viperSlideMotor.getCurrentPosition() < 0 && opModeIsActive()) {
            sleep(5);
        }
        viperSlideMotor.setPower(0);

//        Arm Down
        armMotor.setPower(-0.4);
        armMotor.setTargetPosition(80);

//        Continue Movement
        PID_rotateRight(45, 0.8);
        moveBackward(13, 0.8);
        lateralRight(26, 0.8);
        moveBackward(4, 1);

//        Pick Up Sample
        viperSlideMotor.setTargetPosition(40);
        intakeServo.setPower(-0.5);
        moveForward(9, 0.25);
        sleep(1000);
        moveBackward(3, 0.35);
        intakeServo.setPower(0);
        lateralLeft(26, 0.8);
        moveForward(15, 0.8);
        PID_rotateLeft(45, 0.8);
        PID_forward(9, 0.6);

//        Arm Up
        armMotor.setPower(0.8);
        armMotor.setTargetPosition(1940);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() < 1940 && opModeIsActive()) {
            sleep(5);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setPower(0.4);

//        Viper Up
        viperSlideMotor.setPower(-0.75);
        viperSlideMotor.setTargetPosition(-2600);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition()>-2600 && opModeIsActive()) {
            sleep(5);
            print("Viper Slide Position", Integer.toString(viperSlideMotor.getCurrentPosition()));
        }
        intakeServo.setPower(0.5);
        sleep(1500);
        intakeServo.setPower(0);

//        Viper Down
        viperSlideMotor.setTargetPosition(0);
        viperSlideMotor.setPower(1);
        while (viperSlideMotor.getCurrentPosition() < 0 && opModeIsActive()) {
            sleep(5);
        }
        viperSlideMotor.setPower(0);
    }
}
