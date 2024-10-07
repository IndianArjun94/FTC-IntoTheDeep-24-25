package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTesting", group = "Learning FTC")
public class ServoTest extends OpMode {

    private Servo intakeServo;
    private Servo armServo;

    @Override
    public void init() {
        intakeServo=hardwareMap.get(Servo.class,"intakeServo");
        armServo=hardwareMap.get(Servo.class,"armServo");
    }


    @Override
    public void loop() {

        float triggerValue = gamepad1.left_trigger;

        if(gamepad1.left_bumper){
            intakeServo.setPosition(0.0);
        }
        else if(gamepad1.right_bumper){
            intakeServo.setPosition(1.0);
        }
        else if(gamepad1.left_trigger != 0.0){
            armServo.setPosition(triggerValue);
        }

        else{
            intakeServo.setPosition(0.5);
            armServo.setPosition(0.5);
        }

    }
}
