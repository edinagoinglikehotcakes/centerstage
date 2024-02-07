package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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

    public Launch(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
        pixelStackAprilTags = new PixelStackAprilTags(robotHardware.hardwareMap);
        pixelStackAprilTags.init();
        detectionTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        angleMovetimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public class LaunchDrone implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return launch();
        }
    }

    public Action LaunchDrone() {
        return new LaunchDrone();
    }

    public class LaunchRest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action LaunchRest() {
        return new LaunchRest();
    }

    public class LaunchAngleWaiting implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action LaunchAngleRest() {
        return new LaunchAngleWaiting();
    }

    public class LaunchAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

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
        }


        // Launch!
        robotHardware.DroneLaunch.setPosition(LAUNCHING_SERVO_POSITION);
        return false;
    }

    private double getLaunchPosition(double range) {
        return ((1 - ((range - TAG_RANGE) / (TAG_RANGE)) *
                (LAUNCH_SERVO_ANGLE)));
    }
}
