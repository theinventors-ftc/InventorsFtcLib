package org.inventors.ftc.robotbase.hardware;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import androidx.annotation.NonNull;

public class MotorExEx extends MotorEx {
    private double MAX_SPEED = 1;
    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public MotorExEx(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE);
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public MotorExEx(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }

    /**
     * Constructs an instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */
    public MotorExEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
    }

    @Override
    public void set(double output) {
        super.set(MathUtils.clamp(output, -MAX_SPEED, MAX_SPEED));
    }

    public void setIntegralBounds(double minIntegral, double maxIntegral) {
        veloController.setIntegrationBounds(minIntegral, maxIntegral);
    }

    public void setMaxPower(int power) {
        this.MAX_SPEED = power;
    }
}
