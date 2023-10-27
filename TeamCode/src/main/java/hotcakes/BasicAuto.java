package hotcakes;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.processors.ImageProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Timer;

@Autonomous
public class BasicAuto extends OpMode {
    MecanumDrive drive;
    private ImageProcessor imageProcessor;
    private VisionPortal.Builder visionPortalBuilder;
    private VisionPortal visionPortal;
    private ImageProcessor.Selected selectedSpike;
    private ElapsedTime runTime;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
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

    }

    @Override
    public void start() {
        selectedSpike = imageProcessor.getSelection();
        telemetry.addData("Spike Identified", selectedSpike);
        telemetry.update();
        runTime.reset();
    }

    @Override
    public void loop() {
        TrajectoryActionBuilder ab = drive.actionBuilder(new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        switch (selectedSpike) {
            case LEFT:
            case MIDDLE:
            case RIGHT:
            case NONE:
            default:
        }

        drive.updatePoseEstimate();
        telemetry.addData("runTime", runTime.seconds());
        telemetry.update();
    }
}

