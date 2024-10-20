package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class HWMapConfig {
    public static final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.8F;
    public static final float ARMROT_SPEED_CAP = 0.3F;

    public static DcMotor leftMotor;
    public static DcMotor rightMotor;
    public static DcMotor armMotor;

    public static Servo armYawServo;
    public static CRServo intakeServo;

    public static void init() {
        HWMapConfig.leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        HWMapConfig.rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        HWMapConfig.armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");

        HWMapConfig.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HWMapConfig.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HWMapConfig.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        HWMapConfig.armYawServo = hardwareMap.get(Servo.class, "armServo");
        HWMapConfig.intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
    }
}
