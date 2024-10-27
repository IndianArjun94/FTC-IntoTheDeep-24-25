package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Testing1", group = "Learning FTC")
public class Malav_Arm_Code extends OpMode {

    private DcMotor arm;

    @Override
    public void init() {

        arm=hardwareMap.get(DcMotor.class,"armRotationMotor");
    }

    @Override
    public void loop() {
        if(gamepad1.y){
            arm.setPower(0.3);
        }
        else if(gamepad1.a){
            arm.setPower(-0.3);
        }
        else{
            arm.setPower(0);
        }

    }
}

