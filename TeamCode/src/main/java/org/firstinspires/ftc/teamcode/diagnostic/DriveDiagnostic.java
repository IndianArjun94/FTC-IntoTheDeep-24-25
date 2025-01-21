package org.firstinspires.ftc.teamcode.diagnostic;

import android.support.v4.app.INotificationSideChannel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name = "Drive Diagnostic", group = "Diagnostic")
public class DriveDiagnostic extends LinearOpMode {

    @Override
    public void runOpMode() {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);
        waitForStart();
        while (true && opModeIsActive()) {
            Commons.odo.update();
            telemetry.addData("ODO: ", Commons.odo.getPosition().getX(DistanceUnit.INCH) + " " + Commons.odo.getPosition().getY(DistanceUnit.INCH) + " " + Commons.odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

    }

//    @Override
//    public void loop() {
//        if (gamepad1.x) {
//            frontLeft.setPower(0.5);
//        } else {
//            frontLeft.setPower(0);
//        }
//
//        if (gamepad1.y) {
//            frontRight.setPower(0.5);
//        } else {
//            frontRight.setPower(0);
//        }
//
//        if (gamepad1.b) {
//            backLeft.setPower(0.5);
//        } else {
//            backLeft.setPower(0);
//        }
//
//        if (gamepad1.a) {
//            backRight.setPower(0.5);
//        } else {
//            backRight.setPower(0);
//        }
//    }
}
