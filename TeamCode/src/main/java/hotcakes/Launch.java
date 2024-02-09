package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * This class contains Actions related to launching the drone.
 */
public class Launch {
    private final double LAUNCHING_SERVO_POSITION = 0.35;
    private final double WAITING_SERVO_POSITION = 0.58;
    private final double LAUNCH_SERVO_ANGLE = 0.5;
    private final double WAITING_SERVO_ANGLE = 0;
    private final double DEFAULT_LAUNCH_RANGE = 72;
    private final double TAG_RANGE = 72;
    private RobotHardware robotHardware;
    private PixelStackAprilTags pixelStackAprilTags;
    private AprilTagDetection detectedTag;
    private ElapsedTime detectionTimer;
    private ElapsedTime angleMovetimer;
    private final int DETECTION_WAIT_SECONDS = 2;
    private final double ANGLE_MOVE_WAIT_MILLI_SECONDS = 2000;
    private boolean initialized = false;
    private LAUNCH_ACTION_STATE launchActionState = LAUNCH_ACTION_STATE.DETECTING_TAG;

    private enum LAUNCH_ACTION_STATE {
        DETECTING_TAG,
        SETTING_ANGLE,
        WAITING_FOR_SERVO,
        LAUNCHING
    }

    /**
     * Constuctor.
     *
     * @param robotHardware - RobotHardware to get camera and servos.
     */
    public Launch(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
        pixelStackAprilTags = new PixelStackAprilTags(robotHardware.hardwareMap);
        pixelStackAprilTags.init();
        detectionTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        angleMovetimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * This is the main class that launches the drone.
     */
    public class LaunchDrone implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return launch();
        }
    }

    /**
     * Action to launch the drone.
     *
     * @return - LaunchDrone object for use in trajectories.
     */
    public Action LaunchDrone() {
        return new LaunchDrone();
    }

    public class LaunchRest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    /**
     * Action used to put launcher is rest position.
     *
     * @return LaunchRest object.
     */
    public Action LaunchRest() {
        return new LaunchRest();
    }

    public class LaunchAngleWaiting implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    /**
     * Puts the launch angle in the rest position.
     *
     * @return LaunchAngleWaiting object.
     */
    public Action LaunchAngleRest() {
        return new LaunchAngleWaiting();
    }

    public class LaunchAngle implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    /**
     * Sets the launch angle.
     * @return LaunchAngle object.
     */
    public Action LaunchAngle() {
        return new LaunchAngle();
    }

    private boolean launch() {
        switch (launchActionState) {
            case DETECTING_TAG:
                detectedTag = pixelStackAprilTags.detectTags();
                if (detectedTag == null && detectionTimer.seconds() < DETECTION_WAIT_SECONDS) {
                    return true;
                }

                launchActionState = LAUNCH_ACTION_STATE.SETTING_ANGLE;
                break;
            case SETTING_ANGLE:
                double launchRange = detectedTag == null ? DEFAULT_LAUNCH_RANGE : detectedTag.ftcPose.range;
                pixelStackAprilTags.disableTagProcessing();
                // Set the launch angle
                robotHardware.LaunchAngle.setPosition(getLaunchPosition(launchRange));
                angleMovetimer.reset();
                launchActionState = LAUNCH_ACTION_STATE.WAITING_FOR_SERVO;
                angleMovetimer.reset();
                return true;
            case WAITING_FOR_SERVO:
                if (angleMovetimer.milliseconds() <= ANGLE_MOVE_WAIT_MILLI_SECONDS) {
                    return true;
                }
                break;
            case LAUNCHING:
                robotHardware.DroneLaunch.setPosition(LAUNCHING_SERVO_POSITION);
                return false;
            default:
                break;
        }
        // Something is wrong if get here, tell Action to stop.
        return false;
    }

    private double getLaunchPosition(double range) {
        return ((1 - ((range - TAG_RANGE) / (TAG_RANGE)) *
                (LAUNCH_SERVO_ANGLE)));
    }
}
