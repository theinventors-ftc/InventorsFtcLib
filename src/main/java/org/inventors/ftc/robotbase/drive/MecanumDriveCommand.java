package org.inventors.ftc.robotbase.drive;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drivetrain;
    private final DoubleSupplier forwardSpeed, strafeSpeed, turnSpeed, heading, slowTrigger, fastTrigger;

    public MecanumDriveCommand(MecanumDriveSubsystem drivetrain, DoubleSupplier forwardSpeed,
                               DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed,
                               DoubleSupplier heading, DoubleSupplier fastTrigger, DoubleSupplier slowTrigger) {
        this.drivetrain = drivetrain;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.turnSpeed = turnSpeed;
        this.heading = heading;
        this.fastTrigger = fastTrigger;
        this.slowTrigger = slowTrigger;
        addRequirements(this.drivetrain);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void execute() {
        drivetrain.drive(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                turnSpeed.getAsDouble(), heading.getAsDouble(), fastTrigger.getAsDouble(), slowTrigger.getAsDouble());
    }
}
