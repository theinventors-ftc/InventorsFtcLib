package org.inventors.ftc.robotbase;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.inventors.ftc.opencvpipelines.TeamPropDetectionPipeline;
import org.inventors.ftc.robotbase.controllers.ForwardControllerSubsystem;
import org.inventors.ftc.robotbase.controllers.HeadingControllerSubsystem;
import org.inventors.ftc.robotbase.controllers.HeadingControllerTargetSubsystem;
import org.inventors.ftc.robotbase.drive.StandardTrackingWheelLocalizer;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.drive.MecanumDriveCommand;
import org.inventors.ftc.robotbase.drive.MecanumDriveSubsystem;
import org.inventors.ftc.robotbase.hardware.Camera;
import org.inventors.ftc.robotbase.hardware.DistanceSensorEx;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.inventors.ftc.robotbase.hardware.IMUEmmulatedSubsystem;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.ArrayList;

public class RobotEx {
    // enum to specify opmode type
    public enum OpModeType {
        TELEOP, AUTO
    }

    public enum Alliance {
        RED,
        BLUE
    }

    protected OpModeType opModeType;
    protected FtcDashboard dashboard;

    protected GamepadExEx driverOp;
    protected GamepadExEx toolOp;

    protected MecanumDriveSubsystem drive = null;
    protected MecanumDriveCommand driveCommand = null;

    public Camera camera;

    protected ForwardControllerSubsystem distanceFollow;
    protected HeadingControllerSubsystem gyroFollow;
    protected HeadingControllerSubsystem cameraFollow;
    protected HeadingControllerTargetSubsystem gyroTargetSubsystem;
    protected final Boolean initCamera;
    protected final Boolean initDistance;

    protected IMUEmmulatedSubsystem gyro;
    protected DistanceSensorEx distanceSensor;

    protected Telemetry telemetry, dashTelemetry;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public RobotEx(HardwareMap hardwareMap, DriveConstants RobotConstants, Telemetry telemetry, GamepadExEx driverOp,
                   GamepadExEx toolOp) {
        this(hardwareMap, RobotConstants, telemetry, driverOp, toolOp, OpModeType.TELEOP, false, false, new Pose2d(0, 0, 0));
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public RobotEx(HardwareMap hardwareMap, DriveConstants RobotConstants, Telemetry telemetry, GamepadExEx driverOp,
                   GamepadExEx toolOp, OpModeType type, Boolean initCamera, Boolean initDistance, Pose2d startingPose
    ) {
        this.initCamera = initCamera;
        this.initDistance = initDistance;

        initCommon(hardwareMap, RobotConstants, telemetry, type, startingPose);

        if (type == OpModeType.TELEOP) {
            initTele(hardwareMap, driverOp, toolOp);
            opModeType = OpModeType.TELEOP;
        } else {
            initAuto(hardwareMap);
            opModeType = OpModeType.AUTO;
        }
    }

    public void initCommon(HardwareMap hardwareMap, DriveConstants RobotConstants, Telemetry telemetry, OpModeType type, Pose2d startingPose) {
        ////////////////////////////////////////// Camera //////////////////////////////////////////
        this.dashboard = FtcDashboard.getInstance();
        if (this.initCamera) camera = new Camera(hardwareMap, dashboard, telemetry, TeamPropDetectionPipeline.Alliance.RED);

        //////////////////////////////////////// Telemetries ///////////////////////////////////////
        this.telemetry = telemetry;
        this.dashTelemetry = dashboard.getTelemetry();

        /////////////////////////////////////////// Drive //////////////////////////////////////////
        drive = new MecanumDriveSubsystem(hardwareMap, telemetry, type, RobotConstants, startingPose);

        //////////////////////////////////////////// IMU ///////////////////////////////////////////
        gyro = new IMUEmmulatedSubsystem(telemetry, getMotors()[0], getMotors()[3], startingPose.getHeading());

        CommandScheduler.getInstance().registerSubsystem(gyro);
    }

    public void initAuto(HardwareMap hardwareMap) {
        //////////////////////////////////////// Drivetrain ////////////////////////////////////////
//        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);

        ////////////////////////// Setup and Initialize Mechanisms Objects /////////////////////////
        initMechanismsAutonomous(hardwareMap);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void initTele(HardwareMap hardwareMap, GamepadExEx driverOp,
                         GamepadExEx toolOp) {
        ///////////////////////////////////////// Gamepads /////////////////////////////////////////
        this.driverOp = driverOp;
        this.toolOp = toolOp;

        //////////////////////////////////////// Drivetrain ////////////////////////////////////////
        driveCommand = new MecanumDriveCommand(drive, this::drivetrainForward,
                this::drivetrainStrafe, this::drivetrainTurn, gyro::getRawYaw,
                () -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                telemetry);

        CommandScheduler.getInstance().registerSubsystem(drive);
        drive.setDefaultCommand(driveCommand);

        /////////////////////////////////////// Gyro Follower //////////////////////////////////////
        driverOp.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(gyro::resetYawValue, gyro));

        gyroTargetSubsystem = new HeadingControllerTargetSubsystem(() -> driverOp.getRightX(), () -> driverOp.getRightY(), telemetry);

        gyroFollow = new HeadingControllerSubsystem(gyro::getYaw,
                gyro::findClosestOrientationTarget, telemetry);
//        new Trigger(() -> driverOp.getRightY() >= 0.8).whenActive(
//                new InstantCommand(() -> gyroFollow.setGyroTarget(180), gyroFollow));
//        new Trigger(() -> driverOp.getRightY() <= -0.8).whenActive(
//                new InstantCommand(() -> gyroFollow.setGyroTarget(0), gyroFollow));
//        new Trigger(() -> driverOp.getRightX() >= 0.8).whenActive(
//                new InstantCommand(() -> gyroFollow.setGyroTarget(-90), gyroFollow));
//        new Trigger(() -> driverOp.getRightX() <= -0.8).whenActive(
//                new InstantCommand(() -> gyroFollow.setGyroTarget(90), gyroFollow));

        new Trigger(() -> gyroTargetSubsystem.getMagnitude() >= 0.7 && gyroFollow.isEnabled() && !distanceFollow.isEnabled()).whileActiveContinuous(
                new InstantCommand(() -> gyroFollow.setGyroTarget(gyroTargetSubsystem.getAngle()), gyroFollow));

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(gyroFollow::toggleState, gyroFollow));

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(drive::setFieldCentric, drive));

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(drive::setRobotCentric, drive));

        if (this.initDistance) {
            distanceSensor = new DistanceSensorEx(hardwareMap, "distance_sensor");
            distanceFollow = new ForwardControllerSubsystem(() -> distanceSensor.getDistance(DistanceUnit.MM), 250, telemetry);

            // Backdrop Aligment
            driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(
                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                            new InstantCommand(drive::setRobotCentric),
                                            new InstantCommand(gyroFollow::enable, gyroFollow),
                                            new InstantCommand(() -> gyroFollow.setGyroTarget(90), gyroFollow)
                                    ),
                                    new InstantCommand(distanceFollow::enable, distanceFollow)
                            )
                    );

            driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenReleased(
                            new ParallelCommandGroup(
                                    new InstantCommand(drive::setFieldCentric),
                                    new InstantCommand(gyroFollow::disable, gyroFollow),
                                    new InstantCommand(distanceFollow::disable, distanceFollow)
                            )
                    );
        }

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
        double factor = distanceFollow.isEnabled() ? 0.3 : 1; // This lowers the max power in backdrop alignment for accuracy
        return driverOp.getLeftX() * factor;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double drivetrainForward() {
        double forwardPower;

        if (distanceFollow.isEnabled()) {
            forwardPower = distanceFollow.calculateOutput();
        } else {
            forwardPower = driverOp.getLeftY();
        }

        return forwardPower;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double drivetrainTurn() {
        double turnPower;

        if (gyroFollow.isEnabled()) {
            turnPower = -gyroFollow.calculateTurn();
        } else {
            turnPower = driverOp.getRightX();
        }

        return turnPower;
    }

    public MotorExEx[] getMotors() {
        return drive.getMotors();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public Telemetry getDashboardTelemetry() {
        return dashTelemetry;
    }
}