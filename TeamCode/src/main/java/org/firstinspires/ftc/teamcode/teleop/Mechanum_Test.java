package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpTest1" , group = "Final TeleOp")
public class Mechanum_Test extends OpMode {

    public final float ARMROT_SPEED_CAP = 0.5F;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor armMotor;
    public DcMotor viperSlideMotor;
    public CRServo intakeServo;

    public boolean armMotorUnlocked = false;
    public float armIdlePosition = 0;

    public void print(String value, String value1) {
        telemetry.addData(value, value1);
        telemetry.update();
    }





    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlide");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        print("ViperSlide Position", Float.toString(viperSlideMotor.getCurrentPosition()));

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
            frontLeft.setPower(-0.3);
            frontRight.setPower(0.3);
            backRight.setPower(0.3);
            backLeft.setPower(-0.3);
        }
        // Turn Right
        else if(gamepad1.right_stick_x > 0){
            frontLeft.setPower(0.3);
            frontRight.setPower(-0.3);
            backRight.setPower(-0.3);
            backLeft.setPower(0.3);
        }

        else{
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }


        //armRotation
        if (gamepad1.y || gamepad2.y) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(ARMROT_SPEED_CAP);
            armMotorUnlocked = true;
        } else if (gamepad1.a || gamepad2.a) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(-ARMROT_SPEED_CAP);
            armMotorUnlocked = true;
        } else {
            if (armMotorUnlocked) {
                armMotorUnlocked = false;
                armIdlePosition = armMotor.getCurrentPosition();
                armMotor.setTargetPosition((int) armIdlePosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.4);
            }
        }

        //viperslide
    }
}

