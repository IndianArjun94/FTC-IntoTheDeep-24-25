package org.firstinspires.ftc.teamcode.autonomous.testing;

import static org.firstinspires.ftc.teamcode.Commons.PID_backward;
import static org.firstinspires.ftc.teamcode.Commons.PID_rotateRight;
import static org.firstinspires.ftc.teamcode.Commons.armMotor;
import static org.firstinspires.ftc.teamcode.Commons.frontRightMotor;
import static org.firstinspires.ftc.teamcode.Commons.intakeServo;
import static org.firstinspires.ftc.teamcode.Commons.lateralLeft;
import static org.firstinspires.ftc.teamcode.Commons.lateralRight;
import static org.firstinspires.ftc.teamcode.Commons.moveBackward;
import static org.firstinspires.ftc.teamcode.Commons.moveForward;
import static org.firstinspires.ftc.teamcode.Commons.viperSlideMotor;

import static java.lang.String.valueOf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name = "Auton_Specimen", group = "ftc")
public class Auton_Specimen extends LinearOpMode {

    /**
     * This code runs with parking

     Commons Functions-
     moveForward(inches,speed*0.8);
     moveBackward(inches,speed*0.8);

     PID_Forward(inches,speed*0.8);
     PID_Backward(inches,speed*0.8);

     PID_rotateLeft(turnAngle, speed*0.8);
     PID_rotateRight(turnAngle, speed*0.8);

     lateralRight(inches,speed*0.8);
     lateralLeft(inches,speed*0.8);

     **/

    @Override
    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);
        waitForStart();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Commons.moveForward(3,0.9);

        armMotor.setPower(0.8);
        armMotor.setTargetPosition(1700);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() < 1700 && opModeIsActive()) {
            sleep(2);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());

        //-600 old position viper slide
        viperSlideMotor.setPower(-0.5);
        viperSlideMotor.setTargetPosition(-520);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition() > -520 && opModeIsActive()) {
            sleep(3);
        }
        viperSlideMotor.setTargetPosition(viperSlideMotor.getCurrentPosition());

        Commons.moveForward(23, 0.8);

        armMotor.setPower(-0.3);
        intakeServo.setPower(-0.3);
        armMotor.setTargetPosition(1000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() > 1000 && opModeIsActive()) {
            sleep(2);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());

        viperSlideMotor.setPower(0.5);
        viperSlideMotor.setTargetPosition(0);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition() > 0 && opModeIsActive()) {
            sleep(3);
            telemetry.addData("Position", viperSlideMotor.getCurrentPosition());
            telemetry.addData("Mode", viperSlideMotor.getMode());
            telemetry.update();
        }
        viperSlideMotor.setTargetPosition(viperSlideMotor.getCurrentPosition());


        moveBackward(4, 0.5*1.25);

        intakeServo.setPower(1);

        Commons.PID_rotateRight(96, 0.6*1.25);
        intakeServo.setPower(0);

        moveForward(43,0.65*1.25);


        Commons.PID_rotateRight(94, 0.6*1.25);

        intakeServo.setPower(0);
        armMotor.setPower(-0.8);
        armMotor.setTargetPosition(240);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() > 240 && opModeIsActive()) {
            sleep(2);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());

        intakeServo.setPower(-1);

        sleep(2000);

        moveForward(24,0.2*1.25);

        armMotor.setPower(-0.8);
        armMotor.setTargetPosition(5);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() > 5 && opModeIsActive()) {
            sleep(2);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());

        moveBackward(3,0.7*1.25);

        intakeServo.setPower(0);

        Commons.PID_rotateRight(84,0.7*1.25);

        armMotor.setPower(0.5);
        armMotor.setTargetPosition(1000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() <  1000 && opModeIsActive()) {
            sleep(2);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());

        Commons.PID_rotateRight(90,0.7*1.25);

        lateralLeft(63,0.6*1.25);

        armMotor.setPower(0.8);
        armMotor.setTargetPosition(1800);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() < 1800 && opModeIsActive()) {
            sleep(2);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());

        //-600 old position viper slide
        viperSlideMotor.setPower(-0.5);
        viperSlideMotor.setTargetPosition(-500);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition() > -500 && opModeIsActive()) {
            sleep(3);
        }
        viperSlideMotor.setTargetPosition(viperSlideMotor.getCurrentPosition());

        moveForward(25,0.3*1.25);

        moveBackward(3,0.7*1.25);

        armMotor.setPower(-0.6);
        armMotor.setTargetPosition(1200);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() > 1200 && opModeIsActive()) {
            sleep(2);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());

        moveBackward(13,0.8*1.25);
        intakeServo.setPower(0);

        Commons.lateralRight(40,0.5*1.25);

        Commons.PID_rotateRight(90,0.6);

        Commons.PID_rotateRight(20,0.5*1.25);

        moveForward(12,0.5);

        armMotor.setPower(-0.6);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() > 1200 && opModeIsActive()) {
            sleep(2);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());

/*
        lateralLeft(30,0.5);

        moveForward(15, 0.9);

        lateralRight(48, 0.5);

        lateralLeft(23, 0.5);

        PID_rotateRight(90, 0.6*1.25);

        lateralLeft(2,0.5);

        sleep(5000)

        /*
         */




        sleep(10000);


    }
}