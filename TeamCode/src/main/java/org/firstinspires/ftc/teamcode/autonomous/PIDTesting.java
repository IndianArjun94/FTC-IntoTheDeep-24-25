package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "PID Testing", group = "Test")
public class PIDTesting {

    public final double ticksPerInch = 41.8;

    public IMU imu;
}
