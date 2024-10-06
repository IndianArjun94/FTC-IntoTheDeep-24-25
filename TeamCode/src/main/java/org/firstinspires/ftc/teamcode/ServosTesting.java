package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Testing1", group = "Learning FTC")
public class ServosTesting extends OpMode {

    private Servo intakeServo;

    @Override
    public void init() {
        intakeServo=hardwareMap.get(Servo.class,"intakeServo");
    }

    @Override
    public void loop() {

        if(gamepad1.left_bumper){
            intakeServo.setPosition(0.0);
        }
        else if(gamepad1.right_bumper){
            intakeServo.setPosition(1.0);
        }

        else{
            intakeServo.setPosition(0.5);
        }

    }
}
