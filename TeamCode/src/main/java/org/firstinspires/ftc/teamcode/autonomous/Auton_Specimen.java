package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Commons.PID_backward;
import static org.firstinspires.ftc.teamcode.Commons.PID_forward;
import static org.firstinspires.ftc.teamcode.Commons.PID_rotateRight;
import static org.firstinspires.ftc.teamcode.Commons.armMotor;
import static org.firstinspires.ftc.teamcode.Commons.backLeftMotor;
import static org.firstinspires.ftc.teamcode.Commons.backRightMotor;
import static org.firstinspires.ftc.teamcode.Commons.claw;
import static org.firstinspires.ftc.teamcode.Commons.clawArm;
import static org.firstinspires.ftc.teamcode.Commons.frontLeftMotor;
import static org.firstinspires.ftc.teamcode.Commons.frontRightMotor;
import static org.firstinspires.ftc.teamcode.Commons.intakeServo;
import static org.firstinspires.ftc.teamcode.Commons.lateralLeft;
import static org.firstinspires.ftc.teamcode.Commons.lateralRight;
import static org.firstinspires.ftc.teamcode.Commons.viperSlideMotor;

import static java.lang.String.valueOf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name = "Auton SPECIMEN", group = "Auton FINAL")
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

        PID_forward(2,0.7);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setPower(1);
        armMotor.setTargetPosition(1200);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() < 1200 && opModeIsActive()) {
            sleep(5);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setPower(0.4);

        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(1);
        backRightMotor.setPower(1);
        backLeftMotor.setPower(1);

        sleep(100);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);


        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);

        sleep(1500);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        clawArm.setPower(-0.8);
        clawArm.setTargetPosition(-510);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1000);

        claw.setPosition(0.3);

        sleep(500);

        clawArm.setPower(0.4);
        clawArm.setTargetPosition(-100);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        PID_backward(5, 1.25*0.9);

        PID_rotateRight(90,0.5*1.25);

        PID_forward(34,0.7*1.25);

        lateralLeft(29,0.5*1.25);

        PID_forward(10,0.7*1.25);

        lateralRight(46,0.5*1.25);

        lateralLeft(11,0.5*1.25);

        PID_rotateRight(90,0.7*1.25);

        sleep(500);


        clawArm.setPower(-0.5);
        clawArm.setTargetPosition(-720);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);

        PID_forward(6,0.7);

        PID_forward(3,0.2);

        frontLeftMotor.setPower(0.2);
        frontRightMotor.setPower(0.2);
        backRightMotor.setPower(0.2);
        backLeftMotor.setPower(0.2);

        sleep(150);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        sleep(500);

        claw.setPosition(0.5);

        sleep(500);

        clawArm.setPower(0.5);
        clawArm.setTargetPosition(0);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1000);

        PID_backward(10,0.3);

        PID_rotateRight(85,0.5*1.25);
        PID_rotateRight(90,0.5*1.25);

        lateralLeft(50,0.5*1.25);

        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);

        sleep(1000);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        clawArm.setPower(-0.7);
        clawArm.setTargetPosition(-740);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1000);

        clawArm.setPower(0.4);
        clawArm.setTargetPosition(-100);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1000);

        PID_backward(18,0.7*1.25);

        lateralRight(60,0.5*1.25);


//        clawArm.setPosition(0.4);



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