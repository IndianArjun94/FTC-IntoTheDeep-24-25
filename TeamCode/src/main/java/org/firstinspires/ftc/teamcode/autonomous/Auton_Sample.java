package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Commons.PID_backward;
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
        moveForward(12, 1);
        PID_rotateLeft(90, 1);
        PID_forward(32, 1);
        PID_rotateLeft(45, 1);
        PID_forward(10, 0.8);

//        Arm Up
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setPower(1);
        armMotor.setTargetPosition(2000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() < 2000 && opModeIsActive()) {
            sleep(5);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setPower(0.4);

//        Viper Up
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setPower(-0.95);
        viperSlideMotor.setTargetPosition(-2470);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition()>-2470 && opModeIsActive()) {
            sleep(5);
            print("Viper Slide Position", Integer.toString(viperSlideMotor.getCurrentPosition()));
        }
        intakeServo.setPower(0.3);
        sleep(1500);
        intakeServo.setPower(0);

//        Viper Down
        viperSlideMotor.setTargetPosition(100);
        viperSlideMotor.setPower(1);
        while (viperSlideMotor.getCurrentPosition() < 100 && opModeIsActive()) {
            sleep(5);
        }
        viperSlideMotor.setPower(0);

//        Arm Down
        armMotor.setPower(-0.4);
        armMotor.setTargetPosition(80);

//        Continue Movement
        PID_rotateRight(45, 1);
        PID_backward(10, 0.8);
        sleep(250);
        lateralRight(32, 1);

//        Pick Up Sample
        viperSlideMotor.setTargetPosition(40);
        intakeServo.setPower(-0.5);
        PID_forward(12, 0.3);
        moveBackward(3, 0.35);
        intakeServo.setPower(0);
        lateralLeft(26, 1);
        PID_forward(8, 1);
        PID_rotateLeft(50, 1);
        PID_forward(7, 0.8);

//        Arm Up
        armMotor.setPower(1);
        armMotor.setTargetPosition(1965);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() < 1965 && opModeIsActive()) {
            sleep(5);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setPower(0.4);

//        Viper Up
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setPower(-0.95);
        viperSlideMotor.setTargetPosition(-2470);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition()>-2470 && opModeIsActive()) {
            sleep(5);
            print("Viper Slide Position", Integer.toString(viperSlideMotor.getCurrentPosition()));
        }
        intakeServo.setPower(0.4);
        sleep(1500);
        intakeServo.setPower(0);

//        Viper Down
        viperSlideMotor.setTargetPosition(100);
        viperSlideMotor.setPower(1);
        while (viperSlideMotor.getCurrentPosition() < 100 && opModeIsActive()) {
            sleep(5);
        }
        viperSlideMotor.setPower(0);

//        Arm Down
        armMotor.setPower(-0.4);
        armMotor.setTargetPosition(80);

//        Continue Movement
        PID_rotateRight(45, 1);
        PID_backward(7, 1);
        sleep(350);
        lateralRight(29, 1);

//        Pick Up Sample
        viperSlideMotor.setTargetPosition(40);
        intakeServo.setPower(-0.5);
        PID_forward(17, 0.3);
        moveBackward(6, 0.35);
        intakeServo.setPower(0);
        lateralLeft(26, 1);
//        moveForward(8, 0.8);
        PID_rotateLeft(45, 1);
        PID_forward(7, 0.8);

//        Arm Up
        armMotor.setPower(1);
        armMotor.setTargetPosition(1965);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() < 1965 && opModeIsActive()) {
            sleep(5);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setPower(0.4);

//        Viper Up
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setPower(-0.95);
        viperSlideMotor.setTargetPosition(-2470);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition()>-2470 && opModeIsActive()) {
            sleep(5);
            print("Viper Slide Position", Integer.toString(viperSlideMotor.getCurrentPosition()));
        }
        intakeServo.setPower(0.4);
        sleep(1500);
        intakeServo.setPower(0);

//        Viper Down
        viperSlideMotor.setTargetPosition(100);
        viperSlideMotor.setPower(1);
        while (viperSlideMotor.getCurrentPosition() < 100 && opModeIsActive()) {
            sleep(5);
        }
        viperSlideMotor.setPower(0);
    }
}
