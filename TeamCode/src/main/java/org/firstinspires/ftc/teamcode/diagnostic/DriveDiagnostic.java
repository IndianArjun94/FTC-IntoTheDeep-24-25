package org.firstinspires.ftc.teamcode.diagnostic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Drive Diagnostic", group = "Diagnostic")
public class DriveDiagnostic extends OpMode {
    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.8F;
    public final float ARMROT_SPEED_CAP = 0.5F;

    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor armMotor;

    public float leftMotorSpeed = 0;
    public float rightMotorSpeed = 0;

    public void update() {
//        Robot Movement
        leftMotor.setPower(-leftMotorSpeed*MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        rightMotor.setPower(rightMotorSpeed*MOTOR_MULTIPLIER_PERCENTAGE_CAP);
    }

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
//        Raise Arm & Lock Arm to Position
        armMotor.setPower(0.5);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        armMotor.setPower(0);
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.15);

//
        leftMotor.setPower(MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        rightMotor.setPower(MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
