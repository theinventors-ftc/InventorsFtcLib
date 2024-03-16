package org.inventors.ftc.robotbase.controllers;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class ForwardControllerSubsystem extends SubsystemBase {
    private DoubleSupplier distValue;

    private double distance = 0.0;

    private double target;

    private boolean enabled = false;

    private final double kP = 0.004;
    private final double kI = 0;
    private final double kD = 0.00006;

    PIDFControllerEx controller;

    IIRSubsystem distance_filter;

    private Telemetry telemetry;

    public ForwardControllerSubsystem(DoubleSupplier distValue, double target,
                                      Telemetry telemetry) {
        controller = new PIDFControllerEx(kP, kI, kD, 0, 0, 0, 0.7, 0.05, 0, 0);
        controller.setSetPoint(target);
        this.distValue = distValue;
        this.distance_filter = new IIRSubsystem(0.2, () -> distance);
        this.target = target;
        this.telemetry = telemetry;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void periodic() {
        telemetry.addData("Distance: ", distance);
        telemetry.addData("Distance_Filt: ", distance_filter.get());
    }
    @RequiresApi(api = Build.VERSION_CODES.N)
    public double calculateOutput() {
        distance = distValue.getAsDouble();
        return controller.calculate(distance_filter.get());
    }


    public boolean isEnabled() {
        return enabled;
    }

    public void setGyroTarget(double target) {
        controller.setSetPoint(target);
    }

    public double getTarget() {
        return target;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public void toggleState() {
        enabled = !enabled;
    }
}