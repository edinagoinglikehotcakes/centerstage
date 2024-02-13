package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Drone launching with april tags is managed here.
 */
public class Launch {
    private OpMode opMode;
    private Servo angleServo;
    private Servo laucnhServo;
    private AprilTagDetection detectedTag;
    private final double LAUNCHING_SERVO_POSITION = 0.35;
    private final double WAITING_SERVO_POSITION = 0.58;
    // Default launch range assuming no dynamic angle adjustment.
    private final double DEFAULT_LAUNCH_RANGE = 72;
    // Distance within which the drone can be launched.
    private final double TAG_RANGE = 72;
    // Default launching angle without dynamic angle adjustment.
    private final double LAUNCH_SERVO_ANGLE = 0.5;
    // The waiting angle for the drone launch servo.
    private final double WAITING_SERVO_ANGLE = 0;
    private PixelStackAprilTags pixelStacklAprilTags;
    private ElapsedTime launchTimer;

    private enum LaunchState {
        DETECTING_TAG,
        MOVING_SERVO,
        WAITING_FOR_SEVO_MOVE,
        LAUNCHING
    }

    private LaunchState launchState = LaunchState.DETECTING_TAG;
    private double TAG_DETECT_MILLISECONDS = 1000;
    private double LAUNCH_ANGLE_WAIT_MILLICSECONDS = 1000;

    /***
     * Constructor
     * @param opMode - Used to get hardware map.
     */
    public Launch(OpMode opMode) {
        this.opMode = opMode;
        angleServo = opMode.hardwareMap.get(Servo.class, "launchangle");
        laucnhServo = opMode.hardwareMap.get(Servo.class, "launchservo");
        pixelStacklAprilTags = new PixelStackAprilTags(opMode.hardwareMap);
        launchTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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

    public class LaunchAngleWaiting implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action LaunchAngleWaiting() {
        return new LaunchAngleWaiting();
    }

    public class LaunchDrone implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (launchState) {
                case DETECTING_TAG:
                    launchTimer.reset();
                    if (!isTagDetedted()) {
                        if (launchTimer.milliseconds() >= TAG_DETECT_MILLISECONDS) {
                            launchState = LaunchState.MOVING_SERVO;
                        }
                    }

                    return true;
                case MOVING_SERVO:
                    double launchRange = detectedTag == null ? DEFAULT_LAUNCH_RANGE : detectedTag.ftcPose.range;
                    pixelStacklAprilTags.disableTagProcessing();
                    // Set the launch angle
                    angleServo.setPosition(getLaunchPosition(launchRange));
                    launchState = LaunchState.WAITING_FOR_SEVO_MOVE;
                    return true;
                case WAITING_FOR_SEVO_MOVE:
                    launchTimer.reset();
                    if (launchTimer.milliseconds() >= LAUNCH_ANGLE_WAIT_MILLICSECONDS) {
                        return true;
                    }
                case LAUNCHING:
                    laucnhServo.setPosition(LAUNCHING_SERVO_POSITION);
                    return false;
                default:
                    return true;
            }
        }
    }

    public Action LaunchDrone() {
        return new LaunchDrone();
    }

    private double getLaunchPosition(double range) {
        return ((1 - ((range - TAG_RANGE) / (TAG_RANGE)) *
                (LAUNCH_SERVO_ANGLE)));
    }

    private boolean isTagDetedted() {
        pixelStacklAprilTags = new PixelStackAprilTags(opMode.hardwareMap);
        pixelStacklAprilTags.init();
        detectedTag = pixelStacklAprilTags.detectTags();
        return detectedTag == null ? false : true;
    }
}
