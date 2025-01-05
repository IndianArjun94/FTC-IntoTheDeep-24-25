package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import android.graphics.drawable.PictureDrawable;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.BooleanSupplier;

public class Commons {
    public static final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.5F;
    public static float AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.8f;
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

    public static Telemetry telemetry;

    public static void init(HardwareMap hardwareMap, BooleanSupplier opModeIsActive, Telemetry telemetry) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlide");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT))
        );

        initialized = true;

        Commons.opModeIsActive = opModeIsActive;

        Commons.telemetry = telemetry;
    }

    public static void PID_rotate(double targetTurnAngle) throws InterruptedException {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        imu.resetYaw();

        double currentAngle = Commons.getYawAngle(); // The current angle
        double targetAngle; // The final angle we want to be at when the turn is finished

        targetAngle = targetTurnAngle;

        if (targetAngle < 0.05 && targetAngle > -0.05) {
            return;
        }

        double error = targetAngle - currentAngle; // How much we have to turn to get to targetAngle on the IMU YAW (degrees; right is positive)
        double originalError = targetAngle - currentAngle;

        double P;
        double I = 0;
        double D;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.005;
        double Kd = -0.005;

        double maxI = 0.6;

        while ((error >= -3 || error <= 3) && opModeIsActive.getAsBoolean()) {

            print("Angle", Double.toString(Commons.getYawAngle()));
            print("Error", Double.toString(error));

//            Checks
            if (Math.abs(I) > maxI) {
                I = maxI;
            }
//            PID Calculations
            P = error/originalError * Kp;
            I += error/originalError * Ki;
            D = error/originalError * Kd;
//            Powering Motors
            Commons.startLeft(P+I+D);
//            Update Variables
            error = targetAngle - getYawAngle();
//            Sleep to prevent CPU stress
//            Thread.sleep(5);

        }

        Commons.stopMotors();
    }

    @Deprecated
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
        double Ki = 0.005f;
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
        }

        Commons.stopMotors();
    }

    @Deprecated
    public static void PID_goto(double targetInchesX, double targetInchesY, boolean XcoordFirst) {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        int currentPosition = getMotorPosition(0);
        double targetPositionY = currentPosition + (targetInchesX * ticksPerInch);

        double error = targetPositionY - currentPosition;
        double originalError = targetPositionY - currentPosition;

        double Px;
        double Ix = 0;
        double Dx;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.001f;
        double Kd = 0.08f;

        double Py;
        double Iy = 0;
        double Dy;

        double maxI = 0.3f;

        while ((error != 0) && opModeIsActive.getAsBoolean()) {
//            Checks
            if (Math.abs(Ix) > maxI) {
                Ix = maxI;
            }
//            PID Calculations
            Px = 1-error/originalError * Kp;
            Ix += 1-error/originalError * Ki;
            Dx = 1-error/originalError * Kd;

            Py = error/originalError * Kp;
            Iy += error/originalError * Ki;
            Dy = error/originalError * Kd;

//            Powering Motors
            double PIDy = Py+Iy+Dy;
            double PIDx = Px+Ix+Dx;

            frontLeftMotor.setPower(PIDy + PIDx);
            frontRightMotor.setPower(PIDy - PIDx);
            backLeftMotor.setPower(PIDy + PIDx);
            backRightMotor.setPower(PIDy - PIDx);

//            Updating Variables
            error = targetPositionY - getMotorPosition(0);
        }

        Commons.stopMotors();
    }

//    Robot Values Getters
    public static double getYawAngle() {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return 0;
        }

        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

//        return Math.round(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/10)*10; // Rounds to one decimal place (1234.5678 -> x10 -> 12345.678 -> round -> 12345 -> x0.1 -> 1234.5)
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

    public static void moveForward(int inches, double speed) throws InterruptedException {
        int targetPosition = (int)(inches * ticksPerInch) + getMotorPosition(0);
        setMotorTargetPosition(targetPosition);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        startForward(speed);

        while (frontLeftMotor.isBusy() && opModeIsActive.getAsBoolean()) {
            sleep(1);
        }

        stopMotors();
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

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

    public static void print(String value, String value1) {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        telemetry.addData(value, value1);
        telemetry.update();
    }

    public static void setMotorMode(DcMotor.RunMode runMode) {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        frontLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
    }

    public static void setMotorTargetPosition(int targetPosition) {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return;
        }

        frontLeftMotor.setTargetPosition(targetPosition);
        frontRightMotor.setTargetPosition(targetPosition);
        backLeftMotor.setTargetPosition(targetPosition);
        backRightMotor.setTargetPosition(targetPosition);
    }
}
