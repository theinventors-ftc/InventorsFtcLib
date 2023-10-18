package org.inventors.ftc.robotbase.drivebase;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.inventors.ftc.roadRunner.Localizer;
import org.inventors.ftc.roadRunner.PoseMessage;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import org.inventors.ftc.robotbase.RobotEx;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.inventors.ftc.robotbase.RobotEx.OpModeType.AUTO;
import static org.inventors.ftc.robotbase.RobotEx.OpModeType.TELEOP;

@Config
public class MecanumDriveSubsystem implements Subsystem {
    public static class Params {
        /* ---------------------------------- MOTOR PARAMETERS ---------------------------------- */
        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        public static final double MAX_RPM = 312;

        public static double kP = 1.1;

        public static double KV = 1.0 / rpmToVelocity(MAX_RPM);
        public static double kI = 2.7;
        public static double kD = 0;

        public static double minIntegralBound = -400;
        public static double maxIntegralBound = 400;

        public static Motor.GoBILDA motor_type = Motor.GoBILDA.RPM_312;

        public static double kS_fl = 0;
        public static double kS_rf = 0;
        public static double kS_lb = 0;
        public static double kS_rb = 0;

        public static double kV_fl = 0;
        public static double kV_rf = 0;
        public static double kV_lb = 0;
        public static double kV_rb = 0;
        /* ------------------------------------- AUTONOMOUS ------------------------------------- */

        // drive model parameters
        public double inPerTick = 0;
        public double lateralInPerTick = 1;
        public double trackWidthTicks = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn

        /* ------------------------------------- TELEOP ------------------------------------- */
        public static double rpmToVelocity(double rpm) {
            return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        }

        public static boolean COMMON_FEED_FORWARD = false;

        public static final boolean RUN_USING_ENCODER = true;
        public static boolean frontLeftInverted = true, frontRightInverted = true, rearRightInverted = true, rearLeftInverted = true;

        public static double DEFAULT_SPEED_PERC = 0.6;
        public static double FAST_SPEED_PERC = 1;
        public static double SLOW_SPEED_PERC = 0.3;
        public static double GEAR_RATIO = 0.99639;
        public static double WHEEL_RADIUS = 1.8898;

        public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(19, 0, 9,13.5);
    }

    /*----------------COMMON----------------*/

    private List<MotorExEx> motors;
    private MotorExEx leftFront, rightFront, rightBack, leftBack;

    /*----------------TELEOP----------------*/

    protected String m_name = this.getClass().getSimpleName();
    private com.arcrobotics.ftclib.drivebase.MecanumDrive drive;

    /*--------------------------------------*/
    
    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public VoltageSensor voltageSensor;
    public IMU imu;
    public Localizer localizer;
    public Pose2d pose;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftRear, rightRear, rightFront;

        private int lastLeftFrontPos, lastLeftRearPos, lastRightRearPos, lastRightFrontPos;
        private Rotation2d lastHeading;

        public DriveLocalizer() {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDriveSubsystem.this.leftFront.motorEx));
            leftRear = new OverflowEncoder(new RawEncoder(MecanumDriveSubsystem.this.leftBack.motorEx));
            rightRear = new OverflowEncoder(new RawEncoder(MecanumDriveSubsystem.this.rightBack.motorEx));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDriveSubsystem.this.rightFront.motorEx));

            lastLeftFrontPos = leftFront.getPositionAndVelocity().position;
            lastLeftRearPos = leftRear.getPositionAndVelocity().position;
            lastRightRearPos = rightRear.getPositionAndVelocity().position;
            lastRightFrontPos = rightFront.getPositionAndVelocity().position;

            lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftRearPosVel = leftRear.getPositionAndVelocity();
            PositionVelocityPair rightRearPosVel = rightRear.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double headingDelta = heading.minus(lastHeading);

            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftRearPosVel.position - lastLeftRearPos),
                            leftRearPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightRearPosVel.position - lastRightRearPos),
                            rightRearPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftRearPos = leftRearPosVel.position;
            lastRightRearPos = rightRearPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Pose2d pose, RobotEx.OpModeType type) {
        this.pose = pose;

        /* --------------------------------------- COMMON --------------------------------------- */

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = new MotorExEx(hardwareMap, "frontLeft", PARAMS.motor_type);
        rightFront = new MotorExEx(hardwareMap, "frontRight", PARAMS.motor_type);
        rightBack = new MotorExEx(hardwareMap, "rearRight", PARAMS.motor_type);
        leftBack = new MotorExEx(hardwareMap, "rearLeft", PARAMS.motor_type);

        motors = Arrays.asList(leftFront, rightFront, leftBack, rightBack);

        setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);

        if (!PARAMS.COMMON_FEED_FORWARD) {
            leftFront.setFeedforwardCoefficients(PARAMS.kS_fl, PARAMS.kV_fl, PARAMS.kA);//2795
            rightFront.setFeedforwardCoefficients(PARAMS.kS_rf, PARAMS.kV_rf, PARAMS.kA);//2795
            leftBack.setFeedforwardCoefficients(PARAMS.kS_lb, PARAMS.kV_lb, PARAMS.kA);//2795
            rightBack.setFeedforwardCoefficients(PARAMS.kS_rb, PARAMS.kV_rb, PARAMS.kA);//2795
        } else {
            leftFront.setFeedforwardCoefficients(PARAMS.kS, PARAMS.kV, PARAMS.kA);//2795
            rightFront.setFeedforwardCoefficients(PARAMS.kS, PARAMS.kV, PARAMS.kA);//2795
            leftBack.setFeedforwardCoefficients(PARAMS.kS, PARAMS.kV, PARAMS.kA);//2795
            rightBack.setFeedforwardCoefficients(PARAMS.kS, PARAMS.kV, PARAMS.kA);//2795
        }

        if (type == AUTO) {
            /* ----------------------------------- AUTONOMOUS ----------------------------------- */           

            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);

            voltageSensor = hardwareMap.voltageSensor.iterator().next();

            localizer = new DriveLocalizer();

            FlightRecorder.write("MECANUM_PARAMS", PARAMS);
        } else if (type == TELEOP) {
            /* ------------------------------------- TELEOP ------------------------------------- */
            CommandScheduler.getInstance().registerSubsystem(this);

            if (PARAMS.RUN_USING_ENCODER) setMode(MotorExEx.RunMode.VelocityControl);

            if (PARAMS.RUN_USING_ENCODER && PARAMS.MOTOR_VELO_PID != null) {
                setPIDFCoefficients(PARAMS.kP, PARAMS.kI, PARAMS.kD);
            }

            setMotorsInverted(PARAMS.frontLeftInverted, PARAMS.frontRightInverted, PARAMS.rearRightInverted, PARAMS.rearLeftInverted);
            drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(
                    leftFront, rightFront, leftBack, rightBack
            );
        }

    }


    public void setPIDFCoefficients(double kP, double kI, double kD, double kF, double kV, double kA)
    {
        double batteryPercentage = 12 / voltageSensor.getVoltage();

        for (MotorExEx motor : motors) {
            motor.setIntegralBounds(PARAMS.minIntegralBound, PARAMS.maxIntegralBound);
            motor.setFeedforwardCoefficients(kF * batteryPercentage, kV * batteryPercentage, kA);
            motor.setVeloCoefficients(kP, kI, kD);
        }
    }
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
        drive.setMaxSpeed(PARAMS.DEFAULT_SPEED_PERC + fast_input * PARAMS.FAST_SPEED_PERC - slow_input * PARAMS.SLOW_SPEED_PERC);
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
    }
    public void setMotorsInverted(
            boolean leftFrontInverted, boolean rightFrontInverted,
            boolean rightRearInverted, boolean leftRearInverted
    )
    {
        leftFront.setInverted(leftFrontInverted);
        leftBack.setInverted(leftRearInverted);
        rightFront.setInverted(rightFrontInverted);
        rightBack.setInverted(rightRearInverted);
    }
    public void setMode(Motor.RunMode mode)
    {
        for (MotorExEx motor : motors)
            motor.setRunMode(mode);
    }
    public void setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior zeroPowerBehavior)
    {
        for (MotorExEx motor : motors)
            motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.set(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.set(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.set(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.set(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    (int) Math.ceil(t.path.length() / 2));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.set(0);
                leftBack.set(0);
                rightBack.set(0);
                rightFront.set(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            leftFront.set(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.set(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.set(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.set(feedforward.compute(wheelVels.rightFront) / voltage);

            FlightRecorder.write("TARGET_POSE", new PoseMessage(txWorldTarget.value()));

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.log()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.log()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.set(0);
                leftBack.set(0);
                rightBack.set(0);
                rightFront.set(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            leftFront.set(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.set(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.set(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.set(feedforward.compute(wheelVels.rightFront) / voltage);

            FlightRecorder.write("TARGET_POSE", new PoseMessage(txWorldTarget.value()));

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint,
                0.25, 0.1
        );
    }
}

