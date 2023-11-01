package hotcakes;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

import hotcakes.processors.ImageProcessor;

@Autonomous
public class BasicAuto extends OpMode {
    MecanumDrive drive;
    private ImageProcessor imageProcessor;
    private VisionPortal.Builder visionPortalBuilder;
    private VisionPortal visionPortal;
    private ImageProcessor.Selected selectedSpike;
    private ElapsedTime runTime;
    private AutoState currentAutoState= AutoState.Idle;

    public enum AutoState {
        Idle,
        PlacingPixelOnSpike,
        PlacingPixelInBackstage,
        PlacingPixelOnBackDrop,
        ParkingInBackstage
    }

    @Override
    public void init() {
        //TODO Start left, need menu to offer both start positions
        drive = new MecanumDrive(hardwareMap, new Pose2d(-30, -60, Math.toRadians(90)));

        imageProcessor = new ImageProcessor(telemetry);
        visionPortalBuilder = new VisionPortal.Builder();
        visionPortal = visionPortalBuilder.enableLiveView(true).
                addProcessor(imageProcessor).
                setCamera(hardwareMap.get(WebcamName.class, "webcam1")).
                setCameraResolution(new Size(640, 480)).
                build();

        runTime = new ElapsedTime();
        telemetry.addData("init", "");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        selectedSpike = imageProcessor.getSelection();
        telemetry.addData("Spike Identified", selectedSpike);
        telemetry.update();
    }

    @Override
    public void start() {
        // Save resources
        visionPortal.stopStreaming();
        runTime.reset();
    }

    @Override
    public void loop() {
        switch (selectedSpike) {
            case LEFT:
            case MIDDLE:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .lineToX(30)
                                .lineToX(-5)
                                .turn(Math.toRadians(-90))
                                .lineToX(80)
                                .lineToX(-5)
                                .splineTo(new Vector2d(60, -60), Math.toRadians(90))
                                .build());
                break;
            case RIGHT:
            case NONE:
            default:
        }

        drive.updatePoseEstimate();
        telemetry.addData("runTime", runTime.seconds());
        telemetry.update();
    }
}

