package org.inventors.ftc.robotbase.sensors;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SensorlessStallDetection extends SubsystemBase {

    long timerLenght;
    Timing.Timer timer = new Timing.Timer(timerLenght);

    private boolean stall_detected = false;
    private DoubleSupplier frontLeft;
    private DoubleSupplier frontRight;
    private DoubleSupplier rearLeft;
    private DoubleSupplier rearRight;
    private double stall_current_threshold;
    private double stall_time;

    public SensorlessStallDetection(DoubleSupplier frontLeft, DoubleSupplier frontRight,
                                  DoubleSupplier rearLeft, DoubleSupplier rearRight,
                                  double stall_current_threshold, double stall_time) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.stall_current_threshold = stall_current_threshold;
        this.stall_time = stall_time;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public String stall_detection_processing(DoubleSupplier motor, String motorName){
        timer.start();
        while (true){
            if (motor.getAsDouble() > stall_current_threshold){
                if (timer.elapsedTime() >= stall_time){
                    stall_detected = true;
                    break;
                }
            }
            else break;
        }
        timer.pause();
        return motorName;
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void periodic(){
        if (frontLeft.getAsDouble() > stall_current_threshold){
            stall_detection_processing(frontLeft, "frontLeft");
        }
        if (frontRight.getAsDouble() > stall_current_threshold){
            stall_detection_processing(frontRight, "frontRight");
        }
        if (rearLeft.getAsDouble() > stall_current_threshold){
            stall_detection_processing(rearLeft, "rearLeft");
        }
        if (rearRight.getAsDouble() > stall_current_threshold){
            stall_detection_processing(rearRight, "rearRight");
        }
    }

    public boolean is_stall_detected(){
        return stall_detected;
    }

}