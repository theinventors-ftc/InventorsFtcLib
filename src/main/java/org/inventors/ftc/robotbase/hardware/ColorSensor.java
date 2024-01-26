package org.inventors.ftc.robotbase.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor extends SubsystemBase {
    private NormalizedColorSensor sensor;

    private Boolean lightStatus = false;

    public ColorSensor(HardwareMap hm, String id) {
        this.sensor = hm.get(NormalizedColorSensor.class, id);
    }

    public void setLight(Boolean state) {
        if (sensor instanceof SwitchableLight) {
            lightStatus = !lightStatus;
            ((SwitchableLight) sensor).enableLight(state);
        }
    }

    public void toogleLight() {
        if (sensor instanceof SwitchableLight) {
            lightStatus = !lightStatus;
            ((SwitchableLight) sensor).enableLight(lightStatus);
        }
    }

    public double getDistance() {
        return ((DistanceSensor) sensor).getDistance(DistanceUnit.MM);
    }

    public double getDistance(DistanceUnit unit) {
        return ((DistanceSensor) sensor).getDistance(unit);
    }

    //    Returns a double [0, 1) describing the concentration of the Color Parameter
    public double getColorValue(String color) {
        double value = 0.0;

        switch(color) {
            case "red": value = getRed();
            case "green": value = getGreen();
            case "blue": value = getBlue();
            case "alpha": value = getAlpha();
        }

        return value;
    }

    //    Returns a double [0, 1) describing the concentration of Red Color
    public double getRed() {
        return sensor.getNormalizedColors().red;
    }
    //    Returns a double [0, 1) describing the concentration of Green Color

    public double getGreen() {
        return sensor.getNormalizedColors().green;
    }

    //    Returns a double [0, 1) describing the concentration of Blue Color
    public double getBlue() {
        return sensor.getNormalizedColors().blue;
    }

    //    Returns a double [0, 1) describing the concentration of Alpha Value
    public double getAlpha() {
        return sensor.getNormalizedColors().alpha;
    }
}
