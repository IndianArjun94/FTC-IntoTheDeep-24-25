package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Neel_Mechanum_Wheel_Test")
//@Disabled
public class Neel_Mechanum_Wheel_Test extends OpMode {


    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor armMotor;
    public DcMotor viperSlide;




    public void init() {

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public void telemetryUpdate() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        telemetry.setMsTransmissionInterval(1500);
        telemetry.addData("Viper Slide Position", viperSlide.getCurrentPosition());
        telemetry.addData("Arm Motor Position", viperSlide.getCurrentPosition());
        telemetry.update();
    }




    public void init_loop() {


        //oh yeah what is this??? idk/ using trig to calc


        double leftTriggerPower = gamepad1.left_trigger;
        double rightTriggerPower = -gamepad1.right_trigger;


        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");


        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;


        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);


        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));


        frontLeft.setPower(power * cos/max + turn);
        frontRight.setPower(power * sin/max - turn);
        backLeft.setPower(power * sin/max + turn);
        backRight.setPower(power * cos/max - turn);


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            //do nothing
            int g = 0;
        } else {
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
        }


        if (gamepad1.dpad_up) {
            armMotor.setPower(0.85);
        }
        if (gamepad1.dpad_down) {
            armMotor.setPower(-0.85);
        }


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (leftTriggerPower > 0 || rightTriggerPower > 0) {
            //do nothing
            int g = 0;
        } else {
            viperSlide.setTargetPosition(viperSlide.getCurrentPosition());
        }


        if (leftTriggerPower > 0) {
            viperSlide.setPower(leftTriggerPower);
        }
        if (rightTriggerPower > 0) {
            viperSlide.setPower(rightTriggerPower);
        }


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        telemetryUpdate();


    }


    public void loop() {


        double leftTriggerPower = gamepad1.left_trigger;
        double rightTriggerPower = -gamepad1.right_trigger;


        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");


        double y2 = -gamepad1.left_stick_y;
        double x2 = gamepad1.left_stick_x;


        double turn = gamepad1.right_stick_x;


        double denominator = Math.max(Math.abs(y2) + Math.abs(x2) + Math.abs(turn), 1);


        frontLeft.setPower( (y2 + x2 + turn) / denominator);
        frontRight.setPower( (y2 - x2 + turn) / denominator);
        backLeft.setPower( (y2 - x2 - turn) / denominator);
        backRight.setPower( (y2 + x2 - turn) / denominator);


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            int g = 0;
        } else {
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
        }


        if (gamepad1.dpad_up) {
            armMotor.setPower(0.85);
        }
        if (gamepad1.dpad_down) {
            armMotor.setPower(-0.85);
        }


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (leftTriggerPower > 0 || rightTriggerPower > 0) {
            //do nothing
            int g = 0;
        } else {
            viperSlide.setTargetPosition(viperSlide.getCurrentPosition());
        }


        if (leftTriggerPower > 0) {
            viperSlide.setPower(leftTriggerPower);
        }
        if (rightTriggerPower > 0) {
            viperSlide.setPower(rightTriggerPower);
        }


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        telemetryUpdate();
    }
}
