package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MechTest1" , group = "Final TeleOp")
public class Mechanum_Test extends OpMode {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor armMotor;


    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");
    }

    @Override
    public void loop() {
        // Forward
        if (gamepad1.left_stick_y < 0){
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
        }
        // Backward
        else if(gamepad1.left_stick_y > 0){
            frontLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backRight.setPower(-0.5);
            backLeft.setPower(-0.5);
        }
        // Left
        else if(gamepad1.left_stick_x < 0){
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
        }
        // Right
        else if(gamepad1.left_stick_x > 0){
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
        }
        // Turn Left
        else if(gamepad1.right_stick_x < 0){
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backRight.setPower(0.5);
            backLeft.setPower(-0.5);
        }
        // Turn Right
        else if(gamepad1.right_stick_x > 0){
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backRight.setPower(-0.5);
            backLeft.setPower(0.5);
        }


        else{
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}

