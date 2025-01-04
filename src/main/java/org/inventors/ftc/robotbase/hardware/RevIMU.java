package org.inventors.ftc.robotbase.hardware;

import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.inventors.ftc.robotbase.RobotMapInterface;

@Disabled
public class RevIMU implements HardwareDevice {
    private IMU revIMU;

    /***
     * Heading relative to starting position
     */
    double globalHeading;

    /**
     * Heading relative to last offset
     */
    double relativeHeading;

    /**
     * Offset between global heading and relative heading
     */
    double offset;

    private int multiplier;
    private double yawOffset = 0, pitchOffset = 0, rollOffset = 0;

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub
     *
     * @param robotMap      Robot map
     * @param imuName Name of sensor in configuration
     */
    public RevIMU(RobotMapInterface robotMap) {
//        revIMU = hw.get(IMU.class, imuName);
        revIMU = robotMap.getIMU();
        multiplier = 1;
    }

    /**
     * Initializes gyro with default parameters.
     */
    public void init(double initYaw) {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        init(parameters, initYaw);
    }

    /**
     * Initializes gyro with custom parameters.
     */
    public void init(IMU.Parameters parameters, double initYaw) {
        revIMU.initialize(parameters);

        yawOffset += initYaw;

        resetYaw();
        resetPitch();
        resetRoll();

        globalHeading = 0;
        relativeHeading = 0;
        offset = 0;
    }

    /**
     * Inverts the ouptut of gyro
     */
    public void invertGyro() {
        multiplier *= -1;
    }

    /**
     * @return Absolute heading of the robot
     */

    public void resetYaw() {
        yawOffset = revIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetPitch() {
        pitchOffset = revIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    public void resetRoll() {
        rollOffset = revIMU.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    /**
     * @return X, Y, Z angles of gyro
     */
    public double[] getYawPitchRoll() {
        // make a singular hardware call
        YawPitchRollAngles ypr_angles = revIMU.getRobotYawPitchRollAngles();
        return new double[]{
                ypr_angles.getYaw(AngleUnit.DEGREES) - yawOffset,
                ypr_angles.getPitch(AngleUnit.DEGREES) - pitchOffset,
                ypr_angles.getRoll(AngleUnit.DEGREES) - rollOffset
        };
    }

    public double[] getXYZGs() {
        return new double[3];
    }

    /**
     * @return the internal sensor being wrapped
     */
    public IMU getRevIMU() {
        return revIMU;
    }

    public void disable() {
        revIMU.close();
    }

    @Override
    public String getDeviceType() {
        return "Rev Control Hub IMU";
    }

}
