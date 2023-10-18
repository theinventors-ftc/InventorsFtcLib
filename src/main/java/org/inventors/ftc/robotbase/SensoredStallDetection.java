package org.inventors.ftc.robotbase;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SensoredStallDetection extends SubsystemBase {

    long timerLenght;
    Timing.Timer timer = new Timing.Timer(timerLenght);

    private boolean stall_detected = false;
    private BooleanSupplier frontLeft;
    private BooleanSupplier frontRight;
    private BooleanSupplier rearLeft;
    private BooleanSupplier rearRight;
    private DoubleSupplier frontLeftTicks;
    private DoubleSupplier frontRightTicks;
    private DoubleSupplier rearLeftTicks;
    private DoubleSupplier rearRightTicks;
    private double threshold;

    public SensoredStallDetection(BooleanSupplier frontLeft, BooleanSupplier frontRight,
                                    BooleanSupplier rearLeft, BooleanSupplier rearRight,
                                    DoubleSupplier frontLeftTicks, DoubleSupplier frontRightTicks,
                                    DoubleSupplier rearLeftTicks, DoubleSupplier rearRightTicks,
                                    double threshold) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.frontLeftTicks = frontLeftTicks;
        this.frontRightTicks = frontRightTicks;
        this.rearLeftTicks = rearLeftTicks;
        this.rearRightTicks = rearRightTicks;
        this.threshold = threshold;
    }
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void periodic(){
        if(frontLeft.getAsBoolean()){
            if(frontLeftTicks.getAsDouble() < threshold){
                stall_detected = true;
            }
        }
        if(frontRight.getAsBoolean()){
            if(frontRightTicks.getAsDouble() < threshold){
                stall_detected = true;
            }
        }
        if(rearLeft.getAsBoolean()){
            if(rearLeftTicks.getAsDouble() < threshold){
                stall_detected = true;
            }
        }
        if(rearRight.getAsBoolean()){
            if(rearRightTicks.getAsDouble() < threshold){
                stall_detected = true;
            }
        }
    }

    public boolean is_stall_detected(){
        return stall_detected;
    }

}