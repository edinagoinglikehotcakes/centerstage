package hotcakes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Manage april tags behind the pixel stacks.
 */
public class PixelStackAprilTags {
    private static final int LARGE_BLUE_WALL_TAG_ID = 7;
    private static final int SMALL_BLUE_WALL_TAG_ID = 8;
    private static final int LARGE_RED_WALL_TAG_ID = 10;
    private static final int SMALL_RED_WALL_TAG_ID = 9;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private Boolean targetFound = false;

    /**
     * Initializes the april tags and tries to find the tags.
     */
    public PixelStackAprilTags() {
        initAprilTag();
        desiredTag = findWallTag();
    }

    /**
     * Get the distance to the center of either of the wall april tags.
     * Returns the range or 0 if we did not find the tag.
     */
    public double getRangeToWall() {
        return targetFound ? desiredTag.ftcPose.range : 0;
    }

    /**
     * Get the ftcPose to the april tag.
     *
     * @return - Null if we did not find the tag.
     */
    public AprilTagPoseFtc getAprilTagPose() {
        return targetFound ? desiredTag.ftcPose : null;
    }

    private AprilTagDetection findWallTag() {
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((detection.id == LARGE_BLUE_WALL_TAG_ID) || (detection.id == LARGE_RED_WALL_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        return desiredTag;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam1"))
                .addProcessor(aprilTag)
                .build();
    }
}
