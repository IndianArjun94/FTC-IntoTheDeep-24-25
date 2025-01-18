package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.BooleanSupplier;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.odometry.GoBildaPinpointDriver;

import java.util.Locale;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public  static Servo claw;

    public static DcMotor clawArm;

    public static IMU imu;

    public static GoBildaPinpointDriver odo;

    public static final double ticksPerInch = 41.8;

    public static boolean initialized = false;

    public static BooleanSupplier opModeIsActive;

    public static Telemetry telemetry;

    public static boolean isBusy = false;

    public static void init(HardwareMap hardwareMap, BooleanSupplier opModeIsActive, Telemetry telemetry) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlide");
        clawArm = hardwareMap.get(DcMotor.class, "clawArm");
        claw = hardwareMap.get(Servo.class, "claw");

        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT))
        );

//        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        odo.setOffsets(-84.0, -168.0);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();

        initialized = true;

        Commons.opModeIsActive = opModeIsActive;

        Commons.telemetry = telemetry;
    }

    public static int initWarning() {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return 1;
        }
        return 0;
    }

    public static void PID_rotateLeft(double targetTurnAngle, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

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
        double Ki = 0.015;
        double Kd = -0.005;

        double maxI = 0.6;

        while ((error <= -2 || error >= 2) && opModeIsActive.getAsBoolean()) {

//            print("Angle", Double.toString(Commons.getYawAngle()));
            print("Error", Double.toString(error));

//            Checks
            if (Math.abs(I) > maxI) {
                I = maxI - error/originalError * Ki;
            }
//            PID Calculations
            P = error/originalError * Kp;
            I += error/originalError * Ki;
            D = error/originalError * Kd;
//            Powering Motors
            Commons.startRotateLeft((P+I+D)*speed);
//            Update Variables
            error = targetAngle - getYawAngle();
//            Sleep to prevent CPU stress
            Thread.sleep(5);

        }

        Commons.stopMotorsRotateLeft();
        Commons.stopMotors();

        isBusy = false;
    }

    public static void PID_rotateRight(double targetTurnAngle, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        imu.resetYaw();

        double currentAngle = Commons.getYawAngle(); // The current angle
        double targetAngle; // The final angle we want to be at when the turn is finished

        targetAngle = -targetTurnAngle;

        if (targetAngle < 0.05 && targetAngle > -0.05) {
            return;
        }

        double error = targetAngle - currentAngle; // How much we have to turn to get to targetAngle on the IMU YAW (degrees; right is positive)
        double originalError = targetAngle - currentAngle;

        double P;
        double I = 0;
        double D;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.015;
        double Kd = -0.005;

        double maxI = 0.6;

        while ((error <= -2 || error >= 2) && opModeIsActive.getAsBoolean()) {

//            print("Angle", Double.toString(Commons.getYawAngle()));
            print("Error", Double.toString(error));

//            Checks
            if (Math.abs(I) > maxI) {
                I = maxI - error/originalError * Ki;
            }
//            PID Calculations
            P = error/originalError * Kp;
            I += error/originalError * Ki;
            D = error/originalError * Kd;
//            Powering Motors
            Commons.startRotateRight((P+I+D)*speed);
//            Update Variables
            error = targetAngle - getYawAngle();
//            Sleep to prevent CPU stress
            Thread.sleep(5);

        }

        Commons.stopMotorsRotateRight();
        Commons.stopMotors();

        isBusy = false;
    }

    public static void PID_forward(int targetInches, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        int currentPosition = frontRightMotor.getCurrentPosition();
        double targetPosition = currentPosition - (targetInches * ticksPerInch);

        double error = targetPosition - currentPosition;
        double originalError = targetPosition - currentPosition;

        double P;
        double I = 0.1;
        double D;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.02f;
        double Kd = 0.08f;

        double maxI = 0.5f;


        while ((error <= -ticksPerInch || error >= ticksPerInch) && opModeIsActive.getAsBoolean()) {
//            Checks
            if (Math.abs(I) > maxI) {
                I = maxI - error/originalError * Ki;
            }
//            PID Calculations
            P = error/originalError * Kp;
            I += error/originalError * Ki;
            D = error/originalError * Kd;
//            Powering Motors
            Commons.startForward((P+I+D)*speed);
//            Updating Variables
            error = targetPosition - frontRightMotor.getCurrentPosition();
        }


        Commons.stopMotorsForward();
        Commons.stopMotors();

        isBusy = false;
    }

    public static void PID_backward(int targetInches, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        int currentPosition = frontRightMotor.getCurrentPosition();
        double targetPosition = currentPosition - (-targetInches * ticksPerInch);

        double error = targetPosition - currentPosition;
        double originalError = targetPosition - currentPosition;

        double P;
        double I = 0.1;
        double D;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.02f;
        double Kd = 0.08f;

        double maxI = 0.5f;

        while ((error <= -ticksPerInch || error >= ticksPerInch) && opModeIsActive.getAsBoolean()) {
//            Checks
            if (Math.abs(I) > maxI) {
                I = maxI - error/originalError * Ki;
            }
//            PID Calculations
            P = error/originalError * Kp;
            I += error/originalError * Ki;
            D = error/originalError * Kd;
//            Powering Motors
            Commons.startBackward((P+I+D)*speed);
//            Updating Variables
            error = targetPosition - frontRightMotor.getCurrentPosition();
        }


        Commons.stopMotorsBackward();
        Commons.stopMotors();

        isBusy = false;
    }

    @Deprecated
    public static void PID_goto(double targetInchesX, double targetInchesY, boolean XcoordFirst) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

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

//        Commons.lockMotors(75);
        Commons.stopMotors();

        isBusy = false;
    }

//    Robot Values Getters
    public static double getYawAngle() {
        if (initWarning()==1) {return 181;}

        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Deprecated
    public static int getMotorPosition(int motor) {
        if (initWarning()==1) {return 181;}

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
        if (initWarning()==1) {return;}

        isBusy = true;

        int targetPosition = (int)(-inches * ticksPerInch) + frontRightMotor.getCurrentPosition();

        if (frontRightMotor.getCurrentPosition() < targetPosition) {
            while (frontRightMotor.getCurrentPosition() < targetPosition && opModeIsActive.getAsBoolean()) {
                startForward(speed*AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP);
                telemetry.addData("current pos: ", Integer.toString(backRightMotor.getCurrentPosition()));
                telemetry.update();
            }
        } else if (frontRightMotor.getCurrentPosition() > targetPosition) {
            while (frontRightMotor.getCurrentPosition() > targetPosition && opModeIsActive.getAsBoolean()) {
                startForward(speed);
            }
        }

        Commons.stopMotorsForward();
        stopMotors();

        isBusy = false;
    }

    public static void moveBackward(int inches, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        int targetPosition = (int)(inches * ticksPerInch) + frontRightMotor.getCurrentPosition();

        if (frontRightMotor.getCurrentPosition() > targetPosition) {
            while (frontRightMotor.getCurrentPosition() > targetPosition && opModeIsActive.getAsBoolean()) {
                startBackward(-speed*AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP);
                telemetry.addData("current pos: ", Integer.toString(backRightMotor.getCurrentPosition()));
                telemetry.update();
            }
        } else if (frontRightMotor.getCurrentPosition() < targetPosition) {
            while (frontRightMotor.getCurrentPosition() < targetPosition && opModeIsActive.getAsBoolean()) {
                startBackward(speed);
            }
        }

        Commons.stopMotorsBackward();
        stopMotors();

        isBusy = false;
    }

    public static void startForward(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public static void startBackward(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
    }

    public static void startLateralLeft(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(-speed);
    }

    public static void startLateralRight(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(speed);
    }

    public static void startRotateRight(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
    }

    public static void startRotateLeft(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public static void lateralLeft(double inches, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        int targetPosition = (int)(inches * ticksPerInch) + backRightMotor.getCurrentPosition();
        while (backRightMotor.getCurrentPosition() < targetPosition && opModeIsActive.getAsBoolean()) {
            startLateralLeft(speed*AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        }
        Commons.stopMotorsLateralLeft();
        stopMotors();

        isBusy = false;
    }

    public static void lateralRight(double inches, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        int targetPosition = (int)(-inches * ticksPerInch) + backRightMotor.getCurrentPosition();
        while (backRightMotor.getCurrentPosition() > targetPosition && opModeIsActive.getAsBoolean()) {
            startLateralRight(speed*AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        }
        Commons.stopMotorsLateralRight();
        stopMotors();

        isBusy = false;
    }

    public static void stopMotors() {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public static void print(String value, String value1) {
        if (initWarning()==1) {return;}

        telemetry.addData(value, value1);
        telemetry.update();
    }

    @Deprecated
    public static void setMotorMode(DcMotor.RunMode runMode) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
    }

    @Deprecated
    public static void setMotorTargetPosition(int targetPosition) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setTargetPosition(targetPosition);
        frontRightMotor.setTargetPosition(targetPosition);
        backLeftMotor.setTargetPosition(targetPosition);
        backRightMotor.setTargetPosition(targetPosition);
    }

    public static void stopMotorsForward() throws InterruptedException {
        if (initWarning()==1) {return;}

        startForward(-0.4);
        sleep(75);
        stopMotors();
    }

    public static void stopMotorsBackward() throws InterruptedException {
        if (initWarning()==1) {return;}

        startBackward(-0.4);
        sleep(75);
        stopMotors();
    }

    public static void stopMotorsRotateLeft() throws InterruptedException {
        if (initWarning()==1) {return;}

        startRotateLeft(-0.2);
        sleep(75);
        stopMotors();
    }

    public static void stopMotorsRotateRight() throws InterruptedException {
        if (initWarning()==1) {return;}

        startRotateRight(-0.2);
        sleep(75);
        stopMotors();
    }

    public static void stopMotorsLateralLeft() throws InterruptedException {
        if (initWarning()==1) {return;}

        startLateralLeft(-0.4);
        sleep(75);
        stopMotors();
    }

    public static void stopMotorsLateralRight() throws InterruptedException {
        if (initWarning()==1) {return;}

        startLateralRight(-0.4);
        sleep(75);
        stopMotors();
    }
}
