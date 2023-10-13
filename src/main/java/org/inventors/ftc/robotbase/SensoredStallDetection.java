package org.inventors.ftc.robotbase;

public class SensoredStallDetection extends Subsystem{

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
    
    @Override
    public void periodic(){
        if(frontLeft.getAsBoolean()){
            if(frontLeftTicks =< threshold){
                stall_detected = true;
            }
        }
        if(frontRight.getAsBoolean()){
            if(frontRightTicks < threshold){
                stall_detected = true;
            }
        }
        if(rearLeft.getAsBoolean()){
            if(rearLeftTicks < threshold){
                stall_detected = true;
            }
        }
        if(rearRight.getAsBoolean()){
            if(rearRightTicks < threshold){
                stall_detected = true;
            }
        }
    }

    public boolean is_stall_detected(){
        return stall_detected;
    }

}