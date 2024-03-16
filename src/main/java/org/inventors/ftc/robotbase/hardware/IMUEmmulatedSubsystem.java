package org.inventors.ftc.robotbase.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUEmmulatedSubsystem extends SubsystemBase {

    private Telemetry telemetry;

    private double yaw = 0, yawInit = 0;

    private DcMotor leftEncoder, rightEncoder;

    private int leftPos, rightPos, diff;
//    private double imuEmmulated;

    private final double proportion = 0.003237;


    public IMUEmmulatedSubsystem(Telemetry telemetry, MotorExEx leftEncoderMotor,
                                 MotorExEx rightEncoderMotor, double startingHeading) {
        this.telemetry = telemetry;

        leftEncoder = leftEncoderMotor.getRawMotor();
        rightEncoder = rightEncoderMotor.getRawMotor();

        leftPos = leftEncoder.getCurrentPosition()*-1;
        rightPos = rightEncoder.getCurrentPosition();
        diff = rightPos-leftPos;

        yawInit = diff*proportion - startingHeading;
    }

    public void periodic() {
        leftPos = leftEncoder.getCurrentPosition()*-1;
        rightPos = rightEncoder.getCurrentPosition();
        diff = rightPos-leftPos;
        yaw = diff*proportion;

        calculateContinuousValue();
    }

    public double getYaw() {
        return yaw-yawInit;
    }

    public double getRawYaw() {
        return yaw-yawInit;
    }

    public double getPitch() {
        return 0;
    }

    public int findClosestOrientationTarget() {
        int minDistIdx;
        int maxIdx = (int) Math.ceil(yaw / 45);
        if (Math.abs((maxIdx - 1) * 45 - yaw) > Math.abs((maxIdx) * 45 - yaw))
            minDistIdx = maxIdx;
        else
            minDistIdx = maxIdx - 1;

        return minDistIdx * 45;
    }

    private void calculateContinuousValue() {
//        telemetry.addData("Emmulated Yaw", yaw);
    }

    public void resetYawValue() {
        yawInit = yaw;
    }
}