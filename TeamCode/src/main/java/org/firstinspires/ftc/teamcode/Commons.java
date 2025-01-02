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

    public static final double ticksPerInch = 41.8;

    public static boolean initialized = false;

    public static BooleanSupplier opModeIsActive;

    public static void init(HardwareMap hardwareMap, BooleanSupplier opModeIsActive) {
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

        initialized = true;

        Commons.opModeIsActive = opModeIsActive;
    }

    public static void PID_rotate(double targetTurnAngle, boolean usingTurnAngle) throws InterruptedException {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        double currentAngle = Commons.getYawAngle(); // The current angle
        double targetAngle; // The final angle we want to be at when the turn is finished

        if (usingTurnAngle) {
            targetAngle = currentAngle + targetTurnAngle;
        } else {
            targetAngle = targetTurnAngle;
        }

        if (targetAngle < 0.05 && targetAngle > -0.05) {
            return;
        }

        double error = targetAngle - currentAngle; // How much we have to turn to get to targetAngle on the IMU YAW (degrees; right is positive)
        double originalError = targetAngle - currentAngle;

        double P;
        double I = 0;
        double D;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.001;
        double Kd = -0.08;

        double maxI = 0.3;

        while ((error != 0) && opModeIsActive.getAsBoolean()) {
//            Checks
            if (Math.abs(I) > maxI) {
                I = maxI;
            }
//            PID Calculations
            P = error/originalError * Kp;
            I += error/originalError * Ki;
            D = error/originalError * Kd;
//            Powering Motors
            Commons.startRight(P+I+D);
//            Update Variables
            error = targetAngle - Commons.getYawAngle();
//            Sleep to prevent CPU stress
            Thread.sleep(5);
        }

        Commons.stopMotors();
    }

    public static void PID_forward(double targetInches) throws InterruptedException {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        int currentPosition = getMotorPosition(0);
        double targetPosition = currentPosition + (targetInches * ticksPerInch);

        double error = targetPosition - currentPosition;
        double originalError = targetPosition - currentPosition;

        double P;
        double I = 0;
        double D;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.001f;
        double Kd = 0.08f;

        double maxI = 0.3f;

        while ((error != 0) && opModeIsActive.getAsBoolean()) {
//            Checks
            if (Math.abs(I) > maxI) {
                I = maxI;
            }
//            PID Calculations
            P = error/originalError * Kp;
            I = error/originalError * Ki;
            D = error/originalError * Kd;
//            Powering Motors
            Commons.startForward(P+I+D);
//            Updating Variables
            error = targetPosition - getMotorPosition(0);
//            Sleep to prevent CPU stress
            Thread.sleep(5);
        }

        Commons.stopMotors();
    }

//    public static void PID_goto(double targetInchesX, double targetInchesY, boolean XcoordFirst) {
//        if (!initialized) {
//            System.err.println("Initialize commons first! - \"Commons.init();\"");
//            return;
//        }
//
//        int currentPositionX = getMotorPosition(0);
//        int currentPositionY = get
//        double targetPosition = currentPositionX + (targetInches * ticksPerInch);
//
//        double error = targetPosition - currentPositionX;
//        double originalError = targetPosition - currentPositionX;
//
//        double P;
//        double I = 0;
//        double D;
//
//        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
//        double Ki = 0.001f;
//        double Kd = 0.08f;
//
//        double maxI = 0.3f;
//
//        while ((error != 0) && opModeIsActive.getAsBoolean()) {
////            Checks
//            if (Math.abs(I) > maxI) {
//                I = maxI;
//            }
////            PID Calculations
//            P = error/originalError * Kp;
//            I = error/originalError * Ki;
//            D = error/originalError * Kd;
////            Powering Motors
//            Commons.startForward(P+I+D);
////            Updating Variables
//            error = targetPosition - getMotorPosition(0);
////            Sleep to prevent CPU stress
//            Thread.sleep(5);
//        }
//
//        Commons.stopMotors();
//    }

//    Robot Values Getters
    public static double getYawAngle() {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return 0;
        }

        return Math.round(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/10)*10; // Rounds to one decimal place (1234.5678 -> x10 -> 12345.678 -> round -> 12345 -> x0.1 -> 1234.5)
    }

    public static int getMotorPosition(int motor) {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return 0;
        }

        switch (motor) {
            case 0:
                return frontLeftMotor.getCurrentPosition();
            case 1:
                return frontRightMotor.getCurrentPosition();
            case 2:
                return backLeftMotor.getCurrentPosition();
            case 3:
                return backRightMotor.getCurrentPosition();
            case 4:
                return armMotor.getCurrentPosition();
            case 5:
                return viperSlideMotor.getCurrentPosition();
            default:
                System.err.println("\"" + Integer.toString(motor) + "\" is not recognized as a motor; only values 0 - 5 are supported.");
        }

        return 0;
    }

//    Basic Robot Movement

    public static void startForward(double speed) {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public static void startLeft(double speed) {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        frontLeftMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public static void startRight(double speed) {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
    }

    public static void stopMotors() {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

}
