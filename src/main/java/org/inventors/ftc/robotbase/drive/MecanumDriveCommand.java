package org.inventors.ftc.robotbase.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drivetrain;
    private final DoubleSupplier forwardSpeed, strafeSpeed, turnSpeed, heading, slowFastTrigger;

    public MecanumDriveCommand(MecanumDriveSubsystem drivetrain, DoubleSupplier forwardSpeed,
                               DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed,
                               DoubleSupplier heading, DoubleSupplier slowFastTrigger) {
        this.drivetrain = drivetrain;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.turnSpeed = turnSpeed;
        this.heading = heading;
        this.slowFastTrigger = slowFastTrigger;
      
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                turnSpeed.getAsDouble(), heading.getAsDouble(), slowFastTrigger.getAsDouble());
    }
}
