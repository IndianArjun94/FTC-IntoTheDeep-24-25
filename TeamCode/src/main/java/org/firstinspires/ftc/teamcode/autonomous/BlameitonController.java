package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name ="Control or Hardware_4?", group ="Learning FTC")
//@Disabled
public class BlameitonController extends LinearOpMode {

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

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Motor Power", leftMotor.getPower());
        telemetry.update();
        telemetry.addData("Motor Power", leftMotor.getCurrentPosition());

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setPower(0);
        rightMotor.setPower(-1);
        sleep(450);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setPower(0);
        rightMotor.setPower(0);


    }
}


