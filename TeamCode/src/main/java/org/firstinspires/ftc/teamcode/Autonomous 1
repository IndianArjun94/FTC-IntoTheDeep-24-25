package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name ="Autonomous", group ="Learning FTC")
//@Disabled
public class Auton_1 extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor arm;
    private CRServo intakeServo;
    private Servo armServo;


    public void runOpMode(){
        waitForStart();
        leftMotor=hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor=hardwareMap.get(DcMotor.class, "rightMotor");
        arm=hardwareMap.get(DcMotor.class, "armRotationMotor");
        intakeServo=hardwareMap.get(CRServo.class,"intakeServo");
        armServo=hardwareMap.get(Servo.class,"armServo");

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        arm.setPower(0);

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setPower(-1);
        rightMotor.setPower(1);
        sleep(200);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
