package org.inventors.ftc.robotbase.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.util.Timing.Timer;
public class IMUSubsystem extends SubsystemBase {
    private final RevIMU imu;

    private double previousRawYaw = 0;
    private double turns = 0;
    private double rawYaw = 0, rawPitch = 0, rawRoll = 0;
    private double contYaw;
    private Timer timer;
    private double in_time;

    private double[] angles;

    private Telemetry telemetry;

    public IMUSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        imu = new RevIMU(hardwareMap);
        imu.init();
        timer = new Timer(0);
        timer.start();

        this.telemetry = telemetry;
    }

    public void periodic() {
        in_time = timer.elapsedTime();

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
        int minDistIdx;
        int maxIdx = (int) Math.ceil(contYaw / 45);
        if (Math.abs((maxIdx - 1) * 45 - contYaw) > Math.abs((maxIdx) * 45 - contYaw))
            minDistIdx = maxIdx;
        else
            minDistIdx = maxIdx - 1;

        return minDistIdx * 45;
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