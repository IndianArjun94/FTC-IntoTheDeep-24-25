//This code runs with parking


package org.firstinspires.ftc.teamcode.old_autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous
public class Auton_High_Rung_Specimen extends LinearOpMode {


    //   Name Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor armRotationMotor;
    private DcMotor viperSlide;
    private CRServo intakeServo;


    @Override
    public void runOpMode() throws InterruptedException {


        // initialize
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        armRotationMotor = hardwareMap.get(DcMotor.class, "armRotationMotor");
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");


//        armRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        //forward to position
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);
        try {
            Thread.sleep(40);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        //arm up
        armRotationMotor.setPower(0.5);
        sleep(2500);
        int lockingPosition = armRotationMotor.getCurrentPosition();
        armRotationMotor.setTargetPosition(lockingPosition);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotationMotor.setPower(0.4);


        //pulls specimen in, in case
        intakeServo.setPower(0.9);
        sleep(35);
        intakeServo.setPower(0);

        //viperslide out
        viperSlide.setPower(-0.25);
        sleep(400);
        viperSlide.setPower(0);

        //goes forward (robot)
        frontLeft.setPower(0.275);
        frontRight.setPower(0.275);
        backLeft.setPower(0.275);
        backRight.setPower(0.275);
        viperSlide.setPower(-0.235);
        sleep(1800);
        lockingPosition = armRotationMotor.getCurrentPosition();
        armRotationMotor.setTargetPosition(lockingPosition);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotationMotor.setPower(0.4);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        viperSlide.setPower(0);
        armRotationMotor.setPower(0);

        sleep(50);

        //arm down, specimen on bar
        viperSlide.setPower(0.275);
        armRotationMotor.setPower(-0.75);
        sleep(300);
        armRotationMotor.setPower(0);
        viperSlide.setPower(0);

        armRotationMotor.setPower(0.5);
        sleep(25);
        lockingPosition = armRotationMotor.getCurrentPosition();
        armRotationMotor.setTargetPosition(lockingPosition);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotationMotor.setPower(0.4);



        //back to not hit lower bar
        frontLeft.setPower(-0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(-0.5);
        sleep(100);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        sleep(1000);

        //arm down
        armRotationMotor.setPower(-0.5);
        sleep(120);
        armRotationMotor.setPower(0);

        //viper back
        viperSlide.setPower(0.25);
        sleep(900);
        viperSlide.setPower(0);

        //goes to park (back)
        frontLeft.setPower(-0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(-0.5);
        sleep(800);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        //goes to park (forward positioning)
        frontLeft.setPower(0.8);
        frontRight.setPower(0.8);
        backLeft.setPower(0.8);
        backRight.setPower(0.8);
        sleep(80);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        armRotationMotor.setPower(1);
        sleep(200);
        lockingPosition = armRotationMotor.getCurrentPosition();
        armRotationMotor.setTargetPosition(lockingPosition);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotationMotor.setPower(0.4);


//goes to park (right)
        frontLeft.setPower(0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(0.5);
        sleep(2150);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        armRotationMotor.setPower(0);

        //rest
        sleep(5000);

    }
}

        //
        //
        /*

        frontLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(-0.5);
        try {
            Thread.sleep(650);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);
        try {
            Thread.sleep(480);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        //ARM forward
        armRotationMotor.setPower(0.5);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        armRotationMotor.setPower(0);


        //vipeS forward
        viperSlide.setPower(-0.6);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        viperSlide.setPower(0);


        //ARM back
        armRotationMotor.setPower(0.5);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        armRotationMotor.setPower(0);


        //turns left slight
        frontLeft.setPower(0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(-0.5);
        try {
            Thread.sleep(150);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        //forward
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);
        try {
            Thread.sleep(150);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        //turns to make sample straight
        frontLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);




        frontLeft.setPower(0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(-0.5);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        intakeServo.setPower(0.5);
        sleep(600);
        intakeServo.setPower(0);




        //ARM up
        armRotationMotor.setPower(0.5);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        armRotationMotor.setPower(0);


        //vipeS back
        viperSlide.setPower(0.6);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        viperSlide.setPower(0);


        //back
        frontLeft.setPower(-0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(-0.5);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        //turns around
        frontLeft.setPower(0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(-0.5);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        //let plastic sample go
        intakeServo.setPower(-0.5);
        sleep(500);
        intakeServo.setPower(0);


        frontLeft.setPower(0);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(0);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontRight.setPower(0);
        backLeft.setPower(0);


        sleep(7000);


        frontLeft.setPower(0);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontRight.setPower(0);
        backLeft.setPower(0);


        viperSlide.setPower(0.6);
        sleep(300);
        viperSlide.setPower(0);


        intakeServo.setPower(1);
        sleep(500);
        intakeServo.setPower(0);


        //turns around
        frontLeft.setPower(0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(-0.5);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        frontLeft.setPower(-0.5);
        backRight.setPower(-0.5);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        backRight.setPower(0);


        frontLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(-0.5);
        try {
            Thread.sleep(2500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);






        //arm up
        armRotationMotor.setPower(0.5);
        try {
            Thread.sleep(3100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        armRotationMotor.setPower(0);


        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);
        try {
            Thread.sleep(550);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        frontLeft.setPower(-0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(-0.5);
        armRotationMotor.setPower(0.05);
        try {
            Thread.sleep(700);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        armRotationMotor.setPower(0);


        frontLeft.setPower(0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(0.5);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


         */






