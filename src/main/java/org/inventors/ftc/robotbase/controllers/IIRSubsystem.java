package org.inventors.ftc.robotbase.controllers;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class IIRSubsystem extends SubsystemBase {
    DoubleSupplier value;
    double smoothing_constant;
    double smoothed_value;

    public IIRSubsystem(double smoothing_constant, DoubleSupplier value){
        this.value = value;
        this.smoothing_constant = smoothing_constant;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void periodic() {
        smoothed_value = (1-smoothing_constant) * value.getAsDouble() + smoothing_constant * smoothed_value;
    }

    public void set(double new_smoothing_constant){
        smoothing_constant = new_smoothing_constant;
    }

    public void reset(){
        smoothed_value = 0;
    }

    public double get() {
        return smoothed_value;
    }
}
