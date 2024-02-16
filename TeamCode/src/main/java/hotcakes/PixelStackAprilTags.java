package hotcakes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
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
    private HardwareMap thisHardwareMap;

    /**
     * Pass in the hardware map to find the camera for the VisionPortal.
     *
     * @param hardwareMap
     */
    public PixelStackAprilTags(HardwareMap hardwareMap) {
        thisHardwareMap = hardwareMap;
    }

    public void init() {
        initAprilTag();
        // Wait for the camera to be open
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }
    }

    /**
     * Look for tags and return result.
     *
     * @return AprilTagDetection, a tag or null if none detected.
     */
    public AprilTagDetection detectTags() {
        return findWallTag();
    }

    /**
     * Get the distance to the center of either of the wall april tags.
     * Returns the range or 0 if we did not find the tag.
     */
    public double getRangeToWall() {

        return desiredTag != null ? desiredTag.ftcPose.range : 0;
    }

    /**
     * Get the ftcPose to the april tag.
     *
     * @return - Null if we did not find the tag.
     */
    public AprilTagPoseFtc getAprilTagPose() {
        return desiredTag == null ? null : desiredTag.ftcPose;
    }

    /**
     * Get the detected tag;
     *
     * @return AprilTagDetection
     */
    public AprilTagDetection getDetection() {

        return desiredTag;
    }

    /**
     * Disable april tag processing to conserve resources.
     */
    public void disableTagProcessing() {

        visionPortal.setProcessorEnabled(aprilTag, false);
    }

    private AprilTagDetection findWallTag() {
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        desiredTag = null;
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((detection.id == LARGE_BLUE_WALL_TAG_ID) || (detection.id == LARGE_RED_WALL_TAG_ID)) {
                    // Yes, we want to use this tag.
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }
        }

        return desiredTag;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

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
                .setCamera(thisHardwareMap.get(WebcamName.class, "webcam1"))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }
}
