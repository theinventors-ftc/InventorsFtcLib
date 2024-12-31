package org.inventors.ftc.robotbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.controllers.HeadingControllerSubsystem;
import org.inventors.ftc.robotbase.controllers.HeadingControllerTargetSubsystem;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.drive.MecanumDriveCommand;
import org.inventors.ftc.robotbase.drive.MecanumDriveSubsystem;
import org.inventors.ftc.robotbase.hardware.Camera;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.inventors.ftc.robotbase.hardware.IMUSubsystem;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

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

    public RobotEx(HardwareMap hardwareMap, DriveConstants RobotConstants, Telemetry telemetry,
                   GamepadExEx driverOp, GamepadExEx toolOp, OpModeType type, Alliance alliance,
                   String imu_id, Boolean initCamera, Pose2d startingPose
    ) {
        this.initCamera = initCamera;
        this.alliance = alliance;

        initCommon(hardwareMap, RobotConstants, telemetry, type, imu_id, startingPose);

        if (type == OpModeType.TELEOP) {
            initTele(hardwareMap, driverOp, toolOp);
            opModeType = OpModeType.TELEOP;
        } else {
            initAuto(hardwareMap);
            opModeType = OpModeType.AUTO;
        }
    }

    public void initCommon(HardwareMap hardwareMap, DriveConstants RobotConstants,
                           Telemetry telemetry, OpModeType type, String imuId,
                           Pose2d startingPose) {
        // --------------------------------------- Camera --------------------------------------- //
        this.dashboard = FtcDashboard.getInstance();
        if (this.initCamera) {
            camera = new Camera(hardwareMap, dashboard, telemetry);
        }

        // ------------------------------------- Telemetries ------------------------------------ //
        this.telemetry = telemetry;

        // ---------------------------------------- Drive --------------------------------------- //
        drive = new MecanumDriveSubsystem(hardwareMap, telemetry, type, RobotConstants);

        // ----------------------------------------- IMU ---------------------------------------- //
        gyro = new IMUSubsystem(hardwareMap, telemetry, imuId,
                Math.toDegrees(startingPose.getHeading()));

        CommandScheduler.getInstance().registerSubsystem(gyro);
    }

    public void initAuto(HardwareMap hardwareMap) {
        // ------------------------------------- Drivetrain ------------------------------------- //
        // TODO: Only if we use RoadRunner in TeleOP

        // ----------------------- Setup and Initialize Mechanisms Objects ---------------------- //
        initMechanismsAutonomous(hardwareMap);
    }

    public void initTele(HardwareMap hardwareMap, GamepadExEx driverOp,
                         GamepadExEx toolOp) {
        // -------------------------------------- Gamepads -------------------------------------- //
        this.driverOp = driverOp;
        this.toolOp = toolOp;

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
                .whenPressed(new SequentialCommandGroup(
                            new InstantCommand(gyroFollow::enable, gyroFollow),
                            new InstantCommand(() -> gyroFollow.setGyroTarget(-90), gyroFollow)
                ));


        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(gyroFollow::enable, gyroFollow),
                        new InstantCommand(() -> gyroFollow.setGyroTarget(90), gyroFollow)
                ));

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(drive::setFieldCentric, drive));

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(drive::setRobotCentric, drive));

        // ----------------------- Setup and Initialize Mechanisms Objects ---------------------- //
        initMechanismsTeleOp(hardwareMap);
    }

    public void initMechanismsTeleOp(HardwareMap hardwareMap) {
        // should be overridden by child class
    }

    public void initMechanismsAutonomous(HardwareMap hardwareMap) {
        // should be overridden by child class
    }

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

    public MotorExEx[] getMotors() {
        return drive.getMotors();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

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