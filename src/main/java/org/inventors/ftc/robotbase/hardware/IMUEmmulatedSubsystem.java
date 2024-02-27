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

    public IMUEmmulatedSubsystem(HardwareMap hardwareMap, Telemetry telemetry, MotorExEx leftEncoderMotor,
                                 MotorExEx rightEncoderMotor, double startingHeading) {
        this.telemetry = telemetry;

//        leftEncoder = hardwareMap.get(DcMotor.class, "rearRight");
//        rightEncoder = hardwareMap.get(DcMotor.class, "frontLeft");
        leftEncoder = leftEncoderMotor.getRawMotor();
        rightEncoder = rightEncoderMotor.getRawMotor();

        leftPos = leftEncoder.getCurrentPosition()*-1;
        rightPos = rightEncoder.getCurrentPosition();
        diff = rightPos-leftPos;
        yawInit = diff*proportion;

        resetYawValue(startingHeading);

//        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public double getRoll() {
        return 0;
    }

//    public int findClosestOrientationTarget() {
//        double dist, minDist = Math.abs(yaw);
//        int minDistIdx = 0;
//        int maxIdx = (int) Math.ceil(Math.abs(yaw) / 45);
//        for (int i = maxIdx-2; i <= maxIdx-1; ++i) {
//            dist = Math.abs(i * 45 - yaw);
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
    public void resetYawValue(double startingHeading) {
        yawInit = yaw + startingHeading;
    }
}