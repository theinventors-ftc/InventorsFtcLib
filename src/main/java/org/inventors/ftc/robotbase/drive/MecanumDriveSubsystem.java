package org.inventors.ftc.robotbase.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.RobotMapInterface;
import org.inventors.ftc.robotbase.hardware.Battery;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.Arrays;
import java.util.List;

@Config
public class MecanumDriveSubsystem extends SubsystemBase {
    static DriveConstants RobotConstants;
    static Telemetry telemetry;
    private MotorExEx frontLeft, frontRight, rearRight, rearLeft;
    private List<MotorExEx> motors;
    private Battery battery;
    private com.arcrobotics.ftclib.drivebase.MecanumDrive drive;

    private boolean fieldCentricEnabled = true;

    private boolean is_enabled = true;

    public MecanumDriveSubsystem(
            RobotMapInterface robotMap,
            DriveConstants robotConstants
    ) {
        this.RobotConstants = robotConstants;
        this.telemetry = robotMap.getTelemetry();

        this.battery = robotMap.getBattery();

        this.frontLeft = robotMap.getFrontLeftMotor();
        this.frontRight = robotMap.getFrontRightMotor();
        this.rearLeft = robotMap.getRearLeftMotor();
        this.rearRight = robotMap.getRearRightMotor();

        motors = Arrays.asList(frontLeft, frontRight, rearLeft, rearRight);

        if (RobotConstants.RUN_USING_ENCODER) {
            setMode(MotorExEx.RunMode.VelocityControl);
            resetEncoders();
        }

        setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);

        if (RobotConstants.RUN_USING_ENCODER && RobotConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(RobotConstants.VELO_KP, RobotConstants.VELO_KI, RobotConstants.VELO_KD);

            double batteryPercentage = 12 / battery.getVoltage();

            frontLeft.setFeedforwardCoefficients(
                    RobotConstants.frontLeftFeedForward[0],
                    RobotConstants.frontLeftFeedForward[1],
                    RobotConstants.frontLeftFeedForward[2]
            );
            frontRight.setFeedforwardCoefficients(
                    RobotConstants.frontRightFeedForward[0],
                    RobotConstants.frontRightFeedForward[1],
                    RobotConstants.frontRightFeedForward[2]
            );
            rearLeft.setFeedforwardCoefficients(
                    RobotConstants.rearLeftFeedForward[0],
                    RobotConstants.rearLeftFeedForward[1],
                    RobotConstants.rearLeftFeedForward[2]
            );
            rearRight.setFeedforwardCoefficients(
                    RobotConstants.rearRightFeedForward[0],
                    RobotConstants.rearRightFeedForward[1],
                    RobotConstants.rearRightFeedForward[2]
            );
        }

        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        rearLeft.resetEncoder();
        rearRight.resetEncoder();

        CommandScheduler.getInstance().registerSubsystem(this);
        setMotorsInverted(RobotConstants.frontLeftInverted, RobotConstants.frontRightInverted, RobotConstants.rearRightInverted, RobotConstants.rearLeftInverted);
        drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(
                frontLeft, frontRight, rearLeft, rearRight
        );
    }

    void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading, double slow_fast_input)
    {
        if(is_enabled){
            drive.setMaxSpeed(RobotConstants.FAST_SPEED_PERC * slow_fast_input);

            drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, fieldCentricEnabled ? heading : 0);
        } else {
            drive.setMaxSpeed(1);
        }
    }

    public void setMotorsInverted(
            boolean leftFrontInverted, boolean rightFrontInverted,
            boolean rightRearInverted, boolean leftRearInverted
    )
    {
        frontLeft.setInverted(leftFrontInverted);
        rearLeft.setInverted(leftRearInverted);
        frontRight.setInverted(rightFrontInverted);
        rearRight.setInverted(rightRearInverted);
    }
    public void setMode(Motor.RunMode mode)
    {
        for (MotorExEx motor : motors)
            motor.setRunMode(mode);
    }

    public void resetEncoders() {
        for (MotorExEx motor : motors) motor.resetEncoder();
    }

    public void setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior zeroPowerBehavior)
    {
        for (MotorExEx motor : motors)
            motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setPIDFCoefficients(double kP, double kI, double kD)
    {
        for (MotorExEx motor : motors) {
            motor.setIntegralBounds(RobotConstants.minIntegralBound, RobotConstants.maxIntegralBound);
            motor.setVeloCoefficients(kP, kI, kD);
        }
    }

    public MotorExEx[] getMotors() {
        return new MotorExEx[]{frontLeft, frontRight, rearLeft, rearRight};
    }

    public void toggleMode() {
        fieldCentricEnabled = !fieldCentricEnabled;
    }

    public void setFieldCentric() {
        fieldCentricEnabled = true;
    }
    public void setRobotCentric() {
        fieldCentricEnabled = false;
    }

    public void setEnabled(boolean enabled) {
        is_enabled = enabled;
    }
}