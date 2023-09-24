package org.inventors.ftc.robotbase;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class RobotEx {
    // enum to specify opmode type
    public enum OpModeType {
        TELEOP, AUTO
    }

    protected OpModeType opModeType;
    protected TelemetrySubsystem telemetrySubsystem;
    protected FtcDashboard dashboard;

    protected GamepadExEx driverOp;
    protected GamepadExEx toolOp;

    protected MecanumDrivePPV2 drive = null;
    protected MecanumDriveCommand driveCommand = null;

    protected Pose2d staticPoseStorage;

    public Camera camera;

    protected HeadingControllerSubsystem gyroFollow;
    protected HeadingControllerSubsystem cameraFollow;
    protected final Boolean initCamera;

    protected IMUSubsystem gyro;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public RobotEx(HardwareMap hardwareMap, DriveConstants RobotConstants, Telemetry telemetry, GamepadExEx driverOp,
                   GamepadExEx toolOp, Pose2d staticPoseStorage, BooleanSupplier autoStopRequested) throws NoSuchFieldException, IllegalAccessException {
        this(hardwareMap, RobotConstants, telemetry, driverOp, toolOp, OpModeType.TELEOP, false, false, staticPoseStorage, autoStopRequested);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public RobotEx(HardwareMap hardwareMap, DriveConstants RobotConstants, Telemetry telemetry, GamepadExEx driverOp,
                   GamepadExEx toolOp, OpModeType type, Boolean initCamera, Boolean useCameraFollower, Pose2d staticPoseStorage, BooleanSupplier autoStopRequested
    ) throws NoSuchFieldException, IllegalAccessException {
        this.initCamera = initCamera;
        initCommon(hardwareMap, RobotConstants, telemetry, type, staticPoseStorage);
        if (type == OpModeType.TELEOP) {
            initTele(hardwareMap, driverOp, toolOp, useCameraFollower);
            opModeType = OpModeType.TELEOP;
        } else {
            initAuto(hardwareMap, autoStopRequested);
            opModeType = OpModeType.AUTO;
        }
    }

    public void initCommon(HardwareMap hardwareMap, DriveConstants RobotConstants, Telemetry telemetry, OpModeType type, Pose2d staticPoseStorage) {
        //////////////////////////////////////// Telemetries ///////////////////////////////////////
        dashboard = FtcDashboard.getInstance();
        this.telemetrySubsystem = new TelemetrySubsystem(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().registerSubsystem(telemetrySubsystem);

        ////////////////////////////////////////// Camera //////////////////////////////////////////
        if (this.initCamera) {
            camera = new Camera(hardwareMap, dashboard);
//                    () -> this.driverOp.getButton(GamepadKeys.Button.BACK));
        }

        /////////////////////////////////////////// Drive //////////////////////////////////////////
        drive = new MecanumDrivePPV2(hardwareMap, type, RobotConstants);

        /////////////////////////////////////// Pose Storage ///////////////////////////////////////
        this.staticPoseStorage = staticPoseStorage;
    }

    public void initAuto(HardwareMap hardwareMap, BooleanSupplier isStopRequested) {
        //////////////////////////////////////// Drivetrain ////////////////////////////////////////
//        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);

        new Trigger(isStopRequested).whenActive(new InstantCommand());

        ////////////////////////// Setup and Initialize Mechanisms Objects /////////////////////////
        initMechanismsAutonomous(hardwareMap);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void initTele(HardwareMap hardwareMap, GamepadExEx driverOp,
                         GamepadExEx toolOp, Boolean useCameraFollower) {
        ///////////////////////////////////////// Gamepads /////////////////////////////////////////
        this.driverOp = driverOp;
        this.toolOp = toolOp;

        //////////////////////////////////////////// IMU ///////////////////////////////////////////
        gyro = new IMUSubsystem(hardwareMap, staticPoseStorage.getHeading());
        CommandScheduler.getInstance().registerSubsystem(gyro);

        //////////////////////////////////////// Drivetrain ////////////////////////////////////////
//        drive = new MecanumDriveSubsystem(hardwareMap, frontLeftInvert, frontRightInvert,
//                rearLeftInvert, rearRightInvert);
        driveCommand = new MecanumDriveCommand(drive, this::drivetrainForward,
                this::drivetrainStrafe, this::drivetrainTurn, gyro::getRawYaw,
                () -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        CommandScheduler.getInstance().registerSubsystem(drive);
        drive.setDefaultCommand(driveCommand);


        /////////////////////////////////////// Gyro Follower //////////////////////////////////////
        driverOp.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(gyro::resetYawValue, gyro));

        gyroFollow = new HeadingControllerSubsystem(gyro::getYaw,
                gyro::findClosestOrientationTarget);
        new Trigger(() -> driverOp.getRightY() >= 0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(180), gyroFollow));
        new Trigger(() -> driverOp.getRightY() <= -0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(0), gyroFollow));
        new Trigger(() -> driverOp.getRightX() >= 0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(-90), gyroFollow));
        new Trigger(() -> driverOp.getRightX() <= -0.8).whenActive(
                new InstantCommand(() -> gyroFollow.setGyroTarget(90), gyroFollow));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(0)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(180)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(90)));
//        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new InstantCommand(() -> gyroFollow.setGyroTarget(-90)));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(gyroFollow::toggleState, gyroFollow));

        ////////////////////////////////////// Camera Follower /////////////////////////////////////
//        if (initCamera && useCameraFollower) {
//            cameraFollow = new HeadingControllerSubsystem(camera::getObject_x);
//            driverOp.getGamepadButton(GamepadKeys.Button.START)
//                    .whenPressed(new InstantCommand(cameraFollow::toggleState, cameraFollow));
//        }

        ////////////////////////// Setup and Initialize Mechanisms Objects /////////////////////////
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

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double drivetrainTurn() {
        return gyroFollow.isEnabled() ? -gyroFollow.calculateTurn() : driverOp.getRightX();
    }

    private void stopAndSavePose() {
        drive.setMotorPowers(0, 0, 0, 0); // TODO: Check this
        staticPoseStorage = drive.getPoseEstimate();
    }
}