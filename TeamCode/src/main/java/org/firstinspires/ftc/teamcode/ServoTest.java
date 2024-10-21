package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTesting", group = "Learning FTC")
public class ServoTest extends OpMode {

    private CRServo intakeServo;
    private Servo armServo;

    @Override
    public void init() {
        intakeServo=hardwareMap.get(CRServo.class,"intakeServo");
        armServo=hardwareMap.get(Servo.class,"armServo");
    }


    @Override
    public void loop() {

        double position = armServo.getPosition();

        position = Math.max(0.0, Math.min(position, 1.0));

        if(gamepad1.left_bumper){
            intakeServo.setPower(-1.0);
        }
        else if(gamepad1.right_bumper){
            intakeServo.setPower(1.0);
        }
        else if(gamepad1.left_trigger != 0.0){
            position += 0.01;
            armServo.setPosition(position);

        }
        else if(gamepad1.right_trigger != 0.0){
            position -= 0.01;
            armServo.setPosition(position);

        }
        else{
            intakeServo.setPower(0);
        }
    }
}
