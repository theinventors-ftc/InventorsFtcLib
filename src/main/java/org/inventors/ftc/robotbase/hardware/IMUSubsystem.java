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
    private double[] accel;
    private double maxAccelX, maxAccelY, maxAccelZ;
    private double contYaw;
    private Timer timer;
    private double in_time;

    private double[] angles;
//    private TelemetrySubsystem telemetrySubsystem;

    private Telemetry telemetry;

    public IMUSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        imu = new RevIMU(hardwareMap);
        imu.init();
        timer = new Timer(0);
        timer.start();

        this.telemetry = telemetry;

//        telemetrySubsystem.addMonitor("Gyro Yaw", () -> getRawYaw());
//        telemetrySubsystem.addMonitor("Gyro Pitch", () -> getPitch());
//        telemetrySubsystem.addMonitor("Gyro Roll", () -> getRoll());
//        telemetrySubsystem.addMonitor("Continuous Gyro Value", () -> contYaw);
    }

    public void periodic() {
        in_time = timer.elapsedTime();

        angles = imu.getYawPitchRoll();
        rawYaw = angles[0];
        rawPitch = angles[1];
        rawRoll = angles[2];

        calculateContinuousValue();
//        telemetry.addData("Period", timer.elapsedTime()-in_time);
//        telemetry.addData("Hz", 1/(timer.elapsedTime()-in_time));
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

//    public int findClosestOrientationTarget() {
//        double dist, minDist = Math.abs(contYaw);
//        int minDistIdx = 0;
//        int maxIdx = (int) Math.ceil(Math.abs(contYaw) / 45);
//        for (int i = maxIdx-2; i <= maxIdx-1; ++i) {
//            dist = Math.abs(i * 45 - contYaw);
//            if (dist < minDist) {
//                minDistIdx = i;
//                minDist = dist;
//            }
//        }
//
//        return minDistIdx * 45;
//    }

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

//        telemetry.addData("Raw Yaw", rawYaw);
//        telemetry.addData("Cont Yaw", contYaw);
    }

    public void resetYawValue() {
        imu.resetYaw();
    }
}