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
    private final double kD = 0.001;

    private IIRSubsystem gyro_filter;

    PIDFControllerEx controller;

    private Telemetry telemetry;

    public HeadingControllerSubsystem(DoubleSupplier gyroValue,
                                      IntSupplier closestOrientationTarget,
                                      Telemetry telemetry) {
//        kP = 0.06;
        kP = 0.02;
        controller = new PIDFControllerEx(kP, kI, kD, 0, 0, 1, 0, 0);
        this.gyro_filter = new IIRSubsystem(0, gyroValue);
        this.gyroValue = () -> gyro_filter.get();
        this.closestOrientationTarget = closestOrientationTarget;
        fType = Type.GYRO;

        this.telemetry = telemetry;
    }

    public HeadingControllerSubsystem(DoubleSupplier getCameraObject_x) {
        kP = 0.55;
        controller = new PIDFControllerEx(kP, kI, kD, 0, 0.75, 0, 0, 0);
        this.getCameraObject_x = getCameraObject_x;
        fType = Type.CAMERA;
    }
    @Override
    public void periodic() {
        telemetry.addData("Gyro: ", gyroValue.getAsDouble());
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
        int minDistIdx, maxIdx;

        maxIdx = (int) Math.ceil(gyroValueDouble / 360);
        if (Math.abs((maxIdx - 1) * 360 + targetOrient - gyroValueDouble) > Math.abs((maxIdx) * 360 + targetOrient - gyroValueDouble))
            minDistIdx = maxIdx;
        else
            minDistIdx = maxIdx - 1;

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

    public void enable() {
        enabled = true;
        findClosestTarget = enabled || findClosestTarget;
    }

    public void disable() {
        enabled = false;
        findClosestTarget = enabled || findClosestTarget;
    }
}