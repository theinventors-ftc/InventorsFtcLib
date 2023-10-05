package org.inventors.ftc.robotbase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IMUSubsystem extends SubsystemBase {
    private final RevIMU imu;

    private double previousRawYaw = 0;
    private double turns = 0;
    private double rawYaw = 0, rawPitch = 0, rawRoll = 0;
    private double[] accel;
    private double maxAccelX, maxAccelY, maxAccelZ;
    private double contYaw;

    private double[] angles;
//    private TelemetrySubsystem telemetrySubsystem;

    public IMUSubsystem(HardwareMap hardwareMap) {
        imu = new RevIMU(hardwareMap);
        imu.init();

//        telemetrySubsystem.addMonitor("Gyro Yaw", () -> getRawYaw());
//        telemetrySubsystem.addMonitor("Gyro Pitch", () -> getPitch());
//        telemetrySubsystem.addMonitor("Gyro Roll", () -> getRoll());
//        telemetrySubsystem.addMonitor("Continuous Gyro Value", () -> contYaw);
    }

    public void periodic() {
        angles = imu.getYawPitchRoll();
        rawYaw = angles[0];
        rawPitch = angles[1];
        rawRoll = angles[2];

        calculateContinuousValue();
    }

    public double getYaw() {
        return contYaw;
    }

    public double getRawYaw() {
        return rawYaw;
    }

    public double getPitch() {
        return rawPitch;
    }

    public double getRoll() {
        return rawRoll;
    }

    public int findClosestOrientationTarget() {
        double dist, minDist = Math.abs(contYaw);
        int minDistIdx = 0;
        int maxIdx = (int) Math.ceil(Math.abs(contYaw) / 90);
        for (int i = -maxIdx; i <= maxIdx; ++i) {
            dist = Math.abs(i * 90 - contYaw);
            if (dist < minDist) {
                minDistIdx = i;
                minDist = dist;
            }
        }

        return minDistIdx * 90;
    }

    private void calculateContinuousValue() {
        if (Math.abs(rawYaw - previousRawYaw) >= 180)
            turns += (rawYaw > previousRawYaw) ? -1 : 1;

        previousRawYaw = rawYaw;
        contYaw = rawYaw + 360 * turns;
    }

    public void resetYawValue() {
        imu.resetYaw();
    }
}