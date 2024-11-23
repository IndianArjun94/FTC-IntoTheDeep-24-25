package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Program", group = "Final TeleOp")
public class Viper_Slide extends OpMode {

    public DcMotor viperSlide;
    public final int VIPER_SLIDE_MIN = 0;
    public final int VIPER_SLIDE_MAX = 3000;



    @Override
    public void init() {
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlideMotor");
    }

    @Override
    public void loop() {
//        Viper Slide Movement

        if (gamepad1.right_stick_y > 0.0 || gamepad1.left_stick_y > 0.0) {
            viperSlide.setTargetPosition(VIPER_SLIDE_MAX);
            viperSlide.setPower(gamepad1.right_stick_y);
        } else if (gamepad1.right_stick_y < 0.0 || gamepad1.left_stick_y < 0.0) {
            viperSlide.setTargetPosition(VIPER_SLIDE_MIN);
            viperSlide.setPower(gamepad1.right_stick_y);

        } else {
            viperSlide.setPower(0);
        }

    }
}

