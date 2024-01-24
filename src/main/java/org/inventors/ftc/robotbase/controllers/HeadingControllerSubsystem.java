package org.inventors.ftc.robotbase.controllers;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class HeadingControllerSubsystem extends SubsystemBase {
    public enum Type {
        GYRO, CAMERA
    };

    private final Type fType;

    private DoubleSupplier gyroValue;
    private IntSupplier closestOrientationTarget;
    private DoubleSupplier getCameraObject_x;

    private double target = 0;

    private boolean enabled = false;
    private boolean findClosestTarget;

    private final double kP;
    private final double kI = 0;
    private final double kD = 0;

    PIDController controller;

    private Telemetry telemetry;

    public HeadingControllerSubsystem(DoubleSupplier gyroValue,
                                      IntSupplier closestOrientationTarget,
                                      Telemetry telemetry) {
        kP = 0.06;
        controller = new PIDController(kP, kI, kD);
        this.gyroValue = gyroValue;
        this.closestOrientationTarget = closestOrientationTarget;
        fType = Type.GYRO;

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("Gyro: ", gyroValue.getAsDouble());
    }

    public HeadingControllerSubsystem(DoubleSupplier getCameraObject_x) {
        kP = 0.6;
        controller = new PIDController(kP, kI, kD);
        this.getCameraObject_x = getCameraObject_x;
        fType = Type.CAMERA;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double calculateTurn() {
        double curValue;
        if (fType == Type.CAMERA) {
            curValue = getCameraObject_x.getAsDouble();
//            curValue = camera.getPipeline().getElementsAnalogCoordinates()[0];
//            curValue = 0;
        } else {
            if (findClosestTarget) {
                target = closestOrientationTarget.getAsInt();
                findClosestTarget = false;
            }
            curValue = gyroValue.getAsDouble();
        }

        return controller.calculate(curValue);
    }


    public boolean isEnabled() {
        return enabled;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void setGyroTarget(double targetOrient) {
        double gyroValueDouble = gyroValue.getAsDouble();
        double dist, minDist;
        int minDistIdx, maxIdx;

        minDistIdx = 0;
        minDist = Math.abs(targetOrient - gyroValueDouble);
        maxIdx = (int) Math.ceil(Math.abs(gyroValueDouble) / 360);
        for (int i = maxIdx - 2; i <= maxIdx; i++) {
            dist = Math.abs(i * 360 + targetOrient - gyroValueDouble);
            if (dist < minDist) {
                minDistIdx = i;
                minDist = dist;
            }
        }

        target = minDistIdx * 360 + targetOrient;
        controller.setSetPoint(target);
    }

    public double getTarget() {
        return target;
    }

    public void toggleState() {
        enabled = !enabled;
        findClosestTarget = enabled || findClosestTarget;
    }
}