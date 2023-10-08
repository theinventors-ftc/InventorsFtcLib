package org.inventors.ftc.robotbase;

import android.util.Pair;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class TelemetrySubsystem extends SubsystemBase {
    private MultipleTelemetry multiTelemtry; // Driver Hub and FTC Dashboard
    private List<Pair<String, Supplier> > monitors; // Pair<Caption, ValueSupplier>

    public TelemetrySubsystem(Telemetry dhTelemetry, Telemetry dashTelemetry) {
        multiTelemtry = new MultipleTelemetry(dhTelemetry, dashTelemetry);
    }

    public void addMonitor(String caption, Supplier value) {
        monitors.add(new Pair<>(caption, value));
    }

    @Override
    public void periodic() {
        for(Pair<String, Supplier> monitor : monitors) {
            multiTelemtry.addData(monitor.first+":", monitor.second.get());
        }
    }
}
