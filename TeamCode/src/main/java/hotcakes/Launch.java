package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
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
    private final double LAUNCH_WAITING_SERVO_POSITION = 0.58;
    // Default launching angle without dynamic angle adjustment.
    private final double LAUNCH_SERVO_ANGLE = 0.5;
    // The waiting angle for the drone launch servo.
    private final double WAITING_SERVO_ANGLE = 0;
    // Default launch range assuming no dynamic angle adjustment.
    private final double DEFAULT_LAUNCH_RANGE = 72;
    // Distance within which the drone can be launched.
    private final double TAG_RANGE = 72;
    private PixelStackAprilTags pixelStacklAprilTags;
    private ElapsedTime launchTimer;

    private enum LaunchState {
        INITIALIZING,
        DETECTING_TAG,
        MOVING_SERVO,
        WAITING_FOR_SERVO_MOVE,
        LAUNCHING,
        LOWER_LAUNCHER
    }

    private LaunchState launchState = LaunchState.INITIALIZING;
    private double TAG_DETECT_MILLISECONDS = 1500;
    private double LAUNCH_ANGLE_WAIT_MILLICSECONDS = 4000;

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
            angleServo.setPosition(LAUNCH_SERVO_ANGLE);
            return false;
        }
    }

    public Action LaunchAngle() {
        return new LaunchAngle();
    }

    public class LaunchAngleWaiting implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            angleServo.setPosition(WAITING_SERVO_ANGLE);
            return false;
        }
    }

    public Action LaunchAngleWaiting() {
        return new LaunchAngleWaiting();
    }

    public class LaunchWaiting implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            laucnhServo.setPosition(LAUNCH_WAITING_SERVO_POSITION);
            return false;
        }
    }

    public Action LaunchWaiting() {
        return new LaunchWaiting();
    }

    public class LaunchDrone implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (launchState) {
                case INITIALIZING:
                    pixelStacklAprilTags = new PixelStackAprilTags(opMode.hardwareMap);
                    pixelStacklAprilTags.init();
                    launchTimer.reset();
                    launchState = LaunchState.DETECTING_TAG;
                    break;
                case DETECTING_TAG:
                    detectedTag = pixelStacklAprilTags.detectTags();
                    // Keep looking until one is found or timer runs out.
                    if (detectedTag != null || launchTimer.milliseconds() >= TAG_DETECT_MILLISECONDS) {
                        launchState = LaunchState.MOVING_SERVO;
                    }
                    break;
                case MOVING_SERVO:
                    pixelStacklAprilTags.disableTagProcessing();
                    double launchRange = detectedTag == null ? DEFAULT_LAUNCH_RANGE : detectedTag.ftcPose.range;
                    // Set the launch angle
                    angleServo.setPosition(getLaunchPosition(launchRange));
                    launchState = LaunchState.WAITING_FOR_SERVO_MOVE;
                    launchTimer.reset();
                    break;
                case WAITING_FOR_SERVO_MOVE:
                    if (launchTimer.milliseconds() >= LAUNCH_ANGLE_WAIT_MILLICSECONDS) {
                        launchState = LaunchState.LAUNCHING;
                    }
                    break;
                case LAUNCHING:
                    laucnhServo.setPosition(LAUNCHING_SERVO_POSITION);
                    launchState = LaunchState.LOWER_LAUNCHER;
                    break;
                case LOWER_LAUNCHER:
                    Actions.runBlocking(LaunchAngleWaiting());
                    return false;
            }
            return true;
        }
    }

    public Action LaunchDrone() {
        return new LaunchDrone();
    }

    private double getLaunchPosition(double range) {
        return ((1 - ((range - TAG_RANGE) / (TAG_RANGE)) *
                (LAUNCH_SERVO_ANGLE)));
    }
}
