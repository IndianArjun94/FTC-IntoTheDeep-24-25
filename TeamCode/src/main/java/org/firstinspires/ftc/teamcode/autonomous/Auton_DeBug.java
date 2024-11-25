package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name ="Autonomous_2_OFFICIAL", group ="Learning FTC")
//@Disabled
public class Auton_DeBug extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor arm;
    private CRServo intakeServo;
    private Servo armServo;

    public boolean armMotorUnlocked = false;


    public void runOpMode() {
        waitForStart();
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        arm = hardwareMap.get(DcMotor.class, "armRotationMotor");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        armServo = hardwareMap.get(Servo.class, "armServo");

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        arm.setPower(0);

        telemetry.addData("Postion of arm: ", arm.getCurrentPosition());


        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setPower(-0.5);
        rightMotor.setPower(0.5);
        sleep(1200);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        sleep(350);

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setPower(0);
        rightMotor.setPower(-1);
        sleep(450);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setPower(0.5);
        rightMotor.setPower(-0.5);
        sleep(500);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        sleep(500);

        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setPower(1);
        sleep(1000);
        armMotorUnlocked = true;
        double armIdlePosition = arm.getCurrentPosition();
        arm.setTargetPosition((int)armIdlePosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(700);
        arm.setPower(0);

        intakeServo.setPower(-1);
        sleep(600);

    }
}