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

    private final double kP = 0.068;
    private final double kI = 0.3;
    private final double kD = 0.022;
    private double pidf_out;

    PIDFControllerEx controller;

    private Telemetry telemetry;

    public ForwardControllerSubsystem(DoubleSupplier distValue, double target,
                                      Telemetry telemetry) {
        controller = new PIDFControllerEx(kP, kI, kD, 0, target, 0, 0.2, 1.2, 25, 0.7);
        this.distValue = distValue;
        this.target = target;
        this.telemetry = telemetry;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void periodic() {
        telemetry.addData("Distance: ", distance);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double calculateOutput() {
        distance = distValue.getAsDouble();
        pidf_out = controller.calculate(distance);
        return Math.signum(pidf_out)*pidf_out*pidf_out/1.9;
//        return pidf_out*pidf_out*pidf_out/2;
    }

    public boolean isEnabled() {
        return enabled;
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