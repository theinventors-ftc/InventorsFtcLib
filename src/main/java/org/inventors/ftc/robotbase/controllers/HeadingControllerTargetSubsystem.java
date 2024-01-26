package org.inventors.ftc.robotbase.controllers;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class HeadingControllerTargetSubsystem extends SubsystemBase {
    private HeadingControllerSubsystem headingControllerSubsystem;

    private DoubleSupplier stick_x, stick_y;

    private double vectorAngle = 0.0; // Raw Angle of the Vector
    private double vectorMagnitude = 0.0;
    private double angleTarget = 0; // The closest in 0, 45, 90, 135 ... degrees
    private int[] angles = {0, 45, 90, 135, 180, -45, -90, -135, -180};

    private Telemetry telemetry;

    public HeadingControllerTargetSubsystem(DoubleSupplier stick_x, DoubleSupplier stick_y, Telemetry telemetry) {
        this.stick_x = stick_x;
        this.stick_y = stick_y;

        this.telemetry = telemetry;
    }

//    public static int findClosestAngle(int[] targets, int curAngle) {
//        int closestElement = targets[0];
//        int minDifference = Math.abs(curAngle - closestElement);
//
//        for (int i = 1; i < targets.length; i++) {
//            int currentDifference = Math.abs(curAngle - targets[i]);
//            if (currentDifference < minDifference) {
//                minDifference = currentDifference;
//                closestElement = targets[i];
//            }
//        }
//
//        return closestElement;
//    }

    public static int findClosestAngle(double curAngle) {
        int minDistIdx;
        int maxIdx = (int) Math.ceil(curAngle / 45);
        if (Math.abs((maxIdx - 1) * 45 - curAngle) > Math.abs((maxIdx) * 45 - curAngle))
            minDistIdx = maxIdx;
        else
            minDistIdx = maxIdx - 1;

        return minDistIdx * 45;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void periodic() {
        vectorAngle = Math.toDegrees(Math.atan2(-stick_x.getAsDouble(), -stick_y.getAsDouble()));
        vectorMagnitude = Math.sqrt(Math.pow(stick_x.getAsDouble(), 2) + Math.pow(stick_y.getAsDouble(), 2));

        telemetry.addData("Angle: ", vectorAngle);
        telemetry.addData("Magnituqde: ", vectorMagnitude);
        telemetry.addData("Target Angle: ", getAngle());
    }
    public double getAngle() {
        return findClosestAngle(vectorAngle);
    }
    public double getMagnitude() {
        return vectorMagnitude;
    }
}
