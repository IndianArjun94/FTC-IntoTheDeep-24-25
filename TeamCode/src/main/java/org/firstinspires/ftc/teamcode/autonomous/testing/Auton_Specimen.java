package org.firstinspires.ftc.teamcode.autonomous.testing;

import static org.firstinspires.ftc.teamcode.Commons.armMotor;
import static org.firstinspires.ftc.teamcode.Commons.frontRightMotor;
import static org.firstinspires.ftc.teamcode.Commons.intakeServo;
import static org.firstinspires.ftc.teamcode.Commons.viperSlideMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commons;

@Disabled
@Autonomous
public class Auton_Specimen extends LinearOpMode {

    /**
     * This code runs with parking

     Commons Functions-
     moveForward(inches,speed*0.8);
     moveBackward(inches,speed*0.8);
     PID_Forward(inches,speed*0.8);
     PID_Backward(inches,speed*0.8);
     PID_rotateLeft(turnAngle, speed*0.8);
     PID_rotateRight(turnAngle, speed*0.8);

     **/

    @Override
    public void runOpMode() throws InterruptedException {

        Commons.init(hardwareMap, this::opModeIsActive, telemetry);


        waitForStart();

        Commons.moveForward(1,0.9);

        armMotor.setPower(0.8);
        armMotor.setTargetPosition(1690);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.getCurrentPosition() < 1690 && opModeIsActive()) {
            sleep(2);
        }
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        Commons.moveForward(21, 0.9375);


    }
}