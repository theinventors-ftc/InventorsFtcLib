package org.inventors.ftc.robotbase.drive;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drivetrain;
    private final DoubleSupplier forwardSpeed, strafeSpeed, turnSpeed, heading, slowTrigger, fastTrigger;

    private Telemetry telemetry;

    public MecanumDriveCommand(MecanumDriveSubsystem drivetrain, DoubleSupplier forwardSpeed,
                               DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed,
                               DoubleSupplier heading, DoubleSupplier fastTrigger, DoubleSupplier slowTrigger,
                               Telemetry telemetry) {
        this.drivetrain = drivetrain;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.turnSpeed = turnSpeed;
        this.heading = heading;
        this.fastTrigger = fastTrigger;
        this.slowTrigger = slowTrigger;

        this.telemetry = telemetry;
      
        addRequirements(this.drivetrain);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void execute() {
        drivetrain.drive(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                turnSpeed.getAsDouble(), heading.getAsDouble(), fastTrigger.getAsDouble(), slowTrigger.getAsDouble());
    }
}
