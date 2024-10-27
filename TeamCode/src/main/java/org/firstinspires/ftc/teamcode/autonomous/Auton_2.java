package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name ="Autonomous_Official_2", group ="Learning FTC")
//@Disabled
public class Auton_2 extends LinearOpMode {
//    hi this is arjun's edit_

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

        leftMotor.setPower(0.5);
        rightMotor.setPower(-0.5);
        sleep(1450);
        leftMotor.setPower(0);
        rightMotor.setPower(0);


    }
}

