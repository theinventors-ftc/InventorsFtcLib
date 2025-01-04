package org.inventors.ftc.robotbase;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.hardware.Battery;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;
import org.openftc.easyopencv.OpenCvWebcam;

public interface RobotMapInterface {
    // ----------------------------------------- Telemetry -------------------------------------- //
    Telemetry getTelemetry();

    // ------------------------------------ Drivetrain Motors ----------------------------------- //
    MotorExEx getFrontLeftMotor();
    MotorExEx getFrontRightMotor();
    MotorExEx getRearLeftMotor();
    MotorExEx getRearRightMotor();

    // ------------------------------------------ Sensors --------------------------------------- //
    OpenCvWebcam getCamera();
    IMU getIMU();

    // ------------------------------------------ Gamepads -------------------------------------- //
    GamepadExEx getDriverOp();
    GamepadExEx getToolOp();

    // ------------------------------------------- Battery -------------------------------------- //
    Battery getBattery();
}
