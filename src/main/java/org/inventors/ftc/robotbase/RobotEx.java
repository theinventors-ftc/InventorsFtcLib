package org.inventors.ftc.robotbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.controllers.HeadingControllerSubsystem;
import org.inventors.ftc.robotbase.controllers.HeadingControllerTargetSubsystem;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.drive.MecanumDriveCommand;
import org.inventors.ftc.robotbase.drive.MecanumDriveSubsystem;
import org.inventors.ftc.robotbase.hardware.Camera;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.inventors.ftc.robotbase.hardware.IMUSubsystem;

public class RobotEx {
    public enum OpModeType {
        TELEOP, AUTO
    }

    public enum Alliance {
        RED,
        BLUE
    }

    protected OpModeType opModeType;
    protected Alliance alliance;

    protected FtcDashboard dashboard;

    protected GamepadExEx driverOp;
    protected GamepadExEx toolOp;

    protected MecanumDriveSubsystem drive = null;
    protected MecanumDriveCommand driveCommand = null;

    public Camera camera;

    protected HeadingControllerSubsystem gyroFollow;
    protected HeadingControllerTargetSubsystem gyroTargetSubsystem;
    protected final Boolean initCamera;

    protected IMUSubsystem gyro;

    protected Telemetry telemetry;
    public RobotEx(RobotMapInterface robotMapUtil, DriveConstants RobotConstants,
                   OpModeType type, Alliance alliance, Boolean initCamera, Pose2d startingPose
    ) {
        this.initCamera = initCamera;
        this.alliance = alliance;

        initCommon(robotMapUtil, RobotConstants, type, startingPose);

        if (type == OpModeType.TELEOP) {
            initTele(robotMapUtil);
            opModeType = OpModeType.TELEOP;
        } else {
            initAuto(robotMapUtil);
            opModeType = OpModeType.AUTO;
        }
    }

    public void initCommon(RobotMapInterface robotMapUtil, DriveConstants RobotConstants,
                           OpModeType type, Pose2d startingPose) {
        // --------------------------------------- Camera --------------------------------------- //
        this.dashboard = FtcDashboard.getInstance();
        if (this.initCamera) {
            camera = new Camera(robotMapUtil, dashboard);
        }

        // ------------------------------------- Telemetries ------------------------------------ //
        this.telemetry = robotMapUtil.getTelemetry();

        // ---------------------------------------- Drive --------------------------------------- //
        drive = new MecanumDriveSubsystem(robotMapUtil, RobotConstants);

        // ----------------------------------------- IMU ---------------------------------------- //
        gyro = new IMUSubsystem(robotMapUtil, Math.toDegrees(startingPose.getHeading()));

        CommandScheduler.getInstance().registerSubsystem(gyro);
    }

    public void initAuto(RobotMapInterface robotMapUtil) {
        // ------------------------------------- Drivetrain ------------------------------------- //
        // TODO: Only if we use RoadRunner in TeleOP

        // ----------------------- Setup and Initialize Mechanisms Objects ---------------------- //
        initMechanismsAutonomous();
    }

    public void initTele(RobotMapInterface robotMapUtil) {
        // -------------------------------------- Gamepads -------------------------------------- //
        this.driverOp = robotMapUtil.getDriverOp();
        this.toolOp = robotMapUtil.getToolOp();

        // ------------------------------------- Drivetrain ------------------------------------- //
        driveCommand = new MecanumDriveCommand(drive, this::drivetrainForward,
                this::drivetrainStrafe, this::drivetrainTurn, gyro::getRawYaw,
                () -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                telemetry);

        CommandScheduler.getInstance().registerSubsystem(drive);
        drive.setDefaultCommand(driveCommand);

        // ------------------------------------ Gyro Follower ----------------------------------- //
        driverOp.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(gyro::resetYawValue, gyro));

        gyroTargetSubsystem = new HeadingControllerTargetSubsystem(driverOp::getRightX, driverOp::getRightY, telemetry);

        gyroFollow = new HeadingControllerSubsystem(gyro::getYaw,
                gyro::findClosestOrientationTarget, telemetry);

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(gyroFollow::toggleState, gyroFollow));

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(drive::setFieldCentric, drive));

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(drive::setRobotCentric, drive));

        // ----------------------- Setup and Initialize Mechanisms Objects ---------------------- //
//        initMechanismsTeleOp();
    }

    // ------------------------------------- Drive Commands ------------------------------------- //
    public double drivetrainStrafe() {
        return driverOp.getLeftX();
    }

    public double drivetrainForward() {
        return driverOp.getLeftY();
    }

    public double drivetrainTurn() {
        if (gyroFollow.isEnabled()) return -gyroFollow.calculateTurn();

        return driverOp.getRightX();
    }

    // -------------------------------- Mechanisms Initialization ------------------------------- //
    public void initMechanismsAutonomous() {
        // should be overridden by child class
    }

    public void initMechanismsTeleOp() {
        // should be overridden by child class
    }

    // ----------------------------------------- Getters ---------------------------------------- //
    public double getHeading() {
        return gyro.getRawYaw();
    }

    public double getContinuousHeading() {
        return gyro.getYaw();
    }

    public double getHeadingVelocity() {
        return 0.0; // TODO: Implement
    }
}