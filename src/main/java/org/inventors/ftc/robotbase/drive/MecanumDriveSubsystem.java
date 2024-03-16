package org.inventors.ftc.robotbase.drive;

import static org.inventors.ftc.robotbase.RobotEx.OpModeType.TELEOP;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.hardware.MotorExEx;
import org.inventors.ftc.robotbase.RobotEx;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class MecanumDriveSubsystem extends SubsystemBase {
    static DriveConstants RobotConstants;

    static StandardTrackingWheelLocalizer localizer;

    static Telemetry telemetry;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, RobotEx.OpModeType type, DriveConstants robotConstants, Pose2d startingPose) {
        this.RobotConstants = robotConstants;
        this.telemetry = telemetry;

        localizer = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<Integer>(), new ArrayList<Integer>());
        localizer.setPoseEstimate(startingPose);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        frontLeft = new MotorExEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        frontRight = new MotorExEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        rearRight = new MotorExEx(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);
        rearLeft = new MotorExEx(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);

        motors = Arrays.asList(frontLeft, frontRight, rearLeft, rearRight);

        if (RobotConstants.RUN_USING_ENCODER) {
            setMode(MotorExEx.RunMode.VelocityControl);
            resetEncoders();
        }

        setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);

        if (RobotConstants.RUN_USING_ENCODER && RobotConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(RobotConstants.VELO_KP, RobotConstants.VELO_KI, RobotConstants.VELO_KD);

            double batteryPercentage = 12 / batteryVoltageSensor.getVoltage();

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

        if (type == TELEOP) {
            /* ------------------------------------- TELEOP ------------------------------------- */
            CommandScheduler.getInstance().registerSubsystem(this);
            setMotorsInverted(RobotConstants.frontLeftInverted, RobotConstants.frontRightInverted, RobotConstants.rearRightInverted, RobotConstants.rearLeftInverted);
            drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(
                    frontLeft, frontRight, rearLeft, rearRight
            );
        }
    }

    @Override
    public void periodic() {
        localizer.update();
        telemetry.addData("X: ", localizer.getPoseEstimate().getX());
        telemetry.addData("Y: ", localizer.getPoseEstimate().getY());
//        telemetry.addData("Heading (Pose): ", Math.toDegrees(localizer.getPoseEstimate().getHeading()));
    }
  
    /* ----------------------------------------- TELEOP ----------------------------------------- */
    private MotorExEx frontLeft, frontRight, rearRight, rearLeft;
    private IMU imu;
    private List<MotorExEx> motors;
    private VoltageSensor batteryVoltageSensor;
    protected String m_name = this.getClass().getSimpleName();
    private com.arcrobotics.ftclib.drivebase.MecanumDrive drive;

    private boolean fieldCentricEnabled = true;

    public String getName()
    {
        return m_name;
    }
    public void setName(String name)
    {
        m_name = name;
    }
    public String getSubsystem()
    {
        return getName();
    }
    public void setSubsystem(String subsystem)
    {
        setName(subsystem);
    }
    void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading, double fast_input, double slow_input)
    {
        drive.setMaxSpeed(RobotConstants.DEFAULT_SPEED_PERC + fast_input * RobotConstants.FAST_SPEED_PERC - slow_input * RobotConstants.SLOW_SPEED_PERC);

        if (fieldCentricEnabled) {
            drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
        } else {
            drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
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

}