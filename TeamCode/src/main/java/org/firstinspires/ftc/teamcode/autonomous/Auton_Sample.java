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
        int viperOrigin = viperSlideMotor.getCurrentPosition();
        telemetry.setMsTransmissionInterval(100);
        waitForStart();
        PID_forward(12, 1);
        PID_rotateLeft(90, 1);
        PID_forward(8, 1);
        PID_rotateLeft(45, 1);
        PID_forward(11, 0.8);

//        Arm Up
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setPower(1);
        armMotor.setTargetPosition(1990);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() < 1990 && opModeIsActive()) {
            sleep(5);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setPower(0.4);

//        Viper Up
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setPower(-0.95);
        viperSlideMotor.setTargetPosition(viperOrigin-2085);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition()>viperOrigin-2085 && opModeIsActive()) {
            sleep(5);
            print("Viper Slide Position", Integer.toString(viperSlideMotor.getCurrentPosition()));
        }
        intakeServo.setPower(0.45);
        sleep(2000);
        intakeServo.setPower(0);

//        Viper Down
        viperSlideMotor.setTargetPosition(viperOrigin - 250);
        viperSlideMotor.setPower(1);
        while (viperSlideMotor.getCurrentPosition() < viperOrigin - 250 && opModeIsActive()) {
            sleep(5);
        }
        viperSlideMotor.setPower(0);

//        Arm Down
        armMotor.setPower(-0.4);
        armMotor.setTargetPosition(-100);

//        Continue Movement
        PID_rotateRight(45, 1);
        PID_backward(14, 0.8);
        sleep(250);
        lateralRight(22, 1);

//        Pick Up Sample
        sleep(500);
//        viperSlideMotor.setTargetPosition(-220);
//        viperSlideMotor.setPower(0.7);
        intakeServo.setPower(-0.3);
        PID_forward(12, 0.3);
        moveBackward(3, 0.35);
        intakeServo.setPower(0);
        armMotor.setTargetPosition(0);
        lateralLeft(16, 1);
        PID_forward(10, 1);
        PID_rotateLeft(40, 0.8);
        PID_forward(4, 0.8);

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
        viperSlideMotor.setTargetPosition(viperOrigin-2085);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition()>viperOrigin-2085 && opModeIsActive()) {
            sleep(5);
        }
        intakeServo.setPower(0.65);
        sleep(1500);
        intakeServo.setPower(0);

//        Viper Down
        viperSlideMotor.setTargetPosition(viperOrigin - 100);
        viperSlideMotor.setPower(1);
        while (viperSlideMotor.getCurrentPosition() < viperOrigin - 100 && opModeIsActive()) {
            sleep(5);
        }
        viperSlideMotor.setPower(0);

//        Arm Down
        armMotor.setPower(-0.4);
        armMotor.setTargetPosition(-80);

//        Continue Movement
        PID_rotateRight(45, 1);
        PID_backward(5, 1);
        sleep(350);
        lateralRight(24, 1);

//        Pick Up Sample
        viperSlideMotor.setTargetPosition(-220);
        intakeServo.setPower(-0.2);
        PID_forward(17, 0.3);
        moveBackward(6, 0.5);
        intakeServo.setPower(0);
        armMotor.setTargetPosition(0);
        lateralLeft(19, 1);
        PID_forward(2, 0.8);
        PID_rotateLeft(45, 1);
        PID_forward(5, 0.8);

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
        viperSlideMotor.setTargetPosition(viperOrigin-2085);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (viperSlideMotor.getCurrentPosition()>viperOrigin-2085 && opModeIsActive()) {
            sleep(5);
            print("Viper Slide Position", Integer.toString(viperSlideMotor.getCurrentPosition()));
        }
        intakeServo.setPower(0.65);
        sleep(1500);
        intakeServo.setPower(0);

//        Viper Down
        viperSlideMotor.setTargetPosition(viperOrigin - 100);
        viperSlideMotor.setPower(1);
        while (viperSlideMotor.getCurrentPosition() < viperOrigin - 100 && opModeIsActive()) {
            sleep(5);
        }
        viperSlideMotor.setPower(0);
    }
}
