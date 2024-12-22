package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.BooleanSupplier;

public class Commons {
    public static final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.5F;
    public static final float AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.8f;
    public static final float ARMROT_SPEED_CAP = 0.7F;

    public static DcMotor frontLeftMotor;
    public static DcMotor frontRightMotor;
    public static DcMotor backLeftMotor;
    public static DcMotor backRightMotor;
    public static DcMotor armMotor;
    public static DcMotor viperSlideMotor;

    public static CRServo intakeServo;

    public static IMU imu;

    public final double ticksPerInch = 41.8;

    public static void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlide");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT))
        );
    }

    public static void PID_rotate(double targetTurnAngle, BooleanSupplier opModeIsActive) {
        double targetAngle = Commons.getAngle() + targetTurnAngle;
        double error = targetAngle; // How far off from the target angle

        double P;
        double I = 0;
        double D;

        while ((error > 0.3 || error < -0.3) && opModeIsActive.getAsBoolean()) {
//            PID Calculations
            P = error/targetAngle * Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
            I += error/targetAngle * 0.005;
            D = error/targetAngle * -0.08;
//            Powering Motors
            Commons.startLeft(P+I+D);
//            Update Variables
            error = Commons.getAngle() + targetAngle;
        }

        Commons.stopMotors();
    }

    public static double getAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public static void startLeft(double speed) {
        frontLeftMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public static void startRight(double speed) {
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
    }

    public static void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

}
