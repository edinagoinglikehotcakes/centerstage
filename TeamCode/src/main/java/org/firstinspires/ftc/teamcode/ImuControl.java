package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
    IMU control is handled by this class.
 */
public class ImuControl {
    private RobotHardware robotHardware;
    private IMU imu;

    public ImuControl(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public void init() {
        imu = robotHardware.imu;
    }

    //  Get the yaw in degrees.
    public double getAngle() {
        YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles();
        return yawPitchRollAngles.getYaw(AngleUnit.DEGREES);
    }

    // Reset the current robot orientation to 0. Only call this when the robot is at rest.
    public void resetYaw() {
        imu.resetYaw();
    }
}
