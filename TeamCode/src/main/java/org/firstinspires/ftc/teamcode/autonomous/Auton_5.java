package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name ="Autonomous_1_OFFICIAL", group ="Learning FTC")
//@Disabled
public class Auton_5 extends LinearOpMode {

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

        telemetry.addData("Postion of arm: ", arm.getCurrentPosition());

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        arm.setPower(0);
        telemetry.update();

        sleep(30000);
        telemetry.update();

    }
}
