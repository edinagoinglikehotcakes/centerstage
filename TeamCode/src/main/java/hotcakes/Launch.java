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
    private static final double DEFAULT_LAUNCH_RANGE = 72;
    private OpMode opMode;
    private Servo angleServo;
    private Servo laucnhServo;
    private final double LAUNCHING_SERVO_POSITION = 0.35;
    private final double WAITING_SERVO_POSITION = 0.58;
    private final double TAG_RANGE = 72;
    // Launch
    private final double LAUNCH_SERVO_ANGLE = 0.5;
    private final double WAITING_SERVO_ANGLE = 0;
    private PixelStackAprilTags pixelStacklAprilTags;

    /***
     * Constructor
     * @param opMode - Used to get hardware map.
     */
    public Launch(OpMode opMode) {
        this.opMode = opMode;
        angleServo = opMode.hardwareMap.get(Servo.class, "launchangle");
        laucnhServo = opMode.hardwareMap.get(Servo.class, "launchservo");
        pixelStacklAprilTags = new PixelStackAprilTags(opMode.hardwareMap);
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
            return false;
        }
    }

    public Action LaunchDrone() {
        return new LaunchDrone();
    }

    private void launch() {
        pixelStacklAprilTags = new PixelStackAprilTags(opMode.hardwareMap);
        pixelStacklAprilTags.init();
        AprilTagDetection detectedTag = pixelStacklAprilTags.detectTags();
        double launchRange = detectedTag == null ? DEFAULT_LAUNCH_RANGE : detectedTag.ftcPose.range;
        pixelStacklAprilTags.disableTagProcessing();
        // Set the launch angle
        angleServo.setPosition(getLaunchPosition(launchRange));
        // Wait for the servo to move.
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (timer.milliseconds() <= 4000) {
        }

        // Launch!
        laucnhServo.setPosition(LAUNCHING_SERVO_POSITION);
    }

    private double getLaunchPosition(double range) {
        return ((1 - ((range - TAG_RANGE) / (TAG_RANGE)) *
                (LAUNCH_SERVO_ANGLE)));
    }
}
