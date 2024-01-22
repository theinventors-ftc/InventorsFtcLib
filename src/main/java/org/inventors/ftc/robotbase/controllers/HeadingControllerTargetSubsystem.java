package org.inventors.ftc.robotbase.controllers;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class HeadingControllerTargetSubsystem extends SubsystemBase {
    private HeadingControllerSubsystem headingControllerSubsystem;

    private DoubleSupplier stick_x, stick_y;

    private double vectorAngle = 0.0; // Raw Angle of the Vector
    private double vectorMagnitude = 0.0;
    private double angleTarget = 0; // The closest in 0, 45, 90, 135 ... degrees
    private int[] angles = {0, 45, 90, 135, 180, -45, -90, -135, -180};

    public HeadingControllerTargetSubsystem(DoubleSupplier stick_x, DoubleSupplier stick_y) {
        this.headingControllerSubsystem = headingControllerSubsystem;
        this.stick_x = stick_x;
        this.stick_y = stick_y;
    }

    public static int findClosestAngle(int[] targets, int curAngle) {
        int closestElement = targets[0];
        int minDifference = Math.abs(curAngle - closestElement);

        for (int i = 1; i < targets.length; i++) {
            int currentDifference = Math.abs(curAngle - targets[i]);
            if (currentDifference < minDifference) {
                minDifference = currentDifference;
                closestElement = targets[i];
            }
        }

        return closestElement;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void periodic() {
        vectorAngle = Math.atan2(stick_y.getAsDouble(), stick_x.getAsDouble());
        vectorMagnitude = Math.sqrt(Math.pow(stick_x.getAsDouble(), 2) + Math.pow(stick_y.getAsDouble(), 2));
    }
    public double getAngle() {
        return findClosestAngle(angles, (int)vectorAngle);
    }
    public double getMagnitude() {
        return vectorMagnitude;
    }
}
