//package org.inventors.ftc.robotbase;
//
//import android.util.Pair;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//
//import org.checkerframework.common.util.report.qual.ReportOverride;
//import org.firstinspires.ftc.robotcore.external.Supplier;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import java.util.List;
//
//public class TelemetrySubsystem extends SubsystemBase {
//    private Telemetry[] telemetries = {}; // Driver Hub and FTC Dashboard
//    private List<Pair<String, Supplier> > monitors; // Pair<Caption, ValueSupplier>
//
//    public TelemetrySubsystem(Telemetry dhTelemetry,Telemetry dashTelemetry) {
//        telemetries = new Telemetry[]{dhTelemetry, dashTelemetry};
////        telemetries[0] = dhTelemetry;
////        telemetries[1] = dashTelemetry;
//    }
//
//    public void addMonitor(String caption, Supplier value) {
//        monitors.add(new Pair<>(caption, value));
//    }
//
//    @Override
//    public void periodic() {
//        for(Pair<String, Supplier> monitor : monitors) {
//            for(Telemetry telemetry : telemetries) {
//                telemetry.addData(monitor.first+":", monitor.second.get());
//            }
//        }
//    }
//}
