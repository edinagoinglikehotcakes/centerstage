package hotcakes.processors;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Auto EOCV-SIM", group = "Sim")
//@Disabled
public class AutoTestEOCV extends OpMode {
    private ImageProcessor imageProcessor;
    private VisionPortal.Builder visionPortalBuilder;
    private VisionPortal visionPortal;
    private ImageProcessor.Selected selectedSpike;
    private MecanumDrive mecanumDrive;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        imageProcessor = new ImageProcessor(telemetry);
        visionPortalBuilder = new VisionPortal.Builder();
        visionPortal = visionPortalBuilder.enableLiveView(true).
                addProcessor(imageProcessor).
                setCamera(hardwareMap.get(WebcamName.class, "webcam1")).
                setCameraResolution(new Size(640, 480)).
                build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Init Identified", imageProcessor.getSelection());
        telemetry.update();
    }

    @Override
    public void start() {
            Actions.runBlocking(
                    mecanumDrive.actionBuilder(mecanumDrive.pose)
//                        .splineTo(new Vector2d(30,30),Math.PI /2)
//                        .splineTo(new Vector2d(60,0),Math.PI)
                            .turn(Math.toRadians(90))
                            .build());
        selectedSpike = imageProcessor.getSelection();
        telemetry.addData("Start Identified", selectedSpike);
        telemetry.update();
        // Save resources
//        visionPortal.setProcessorEnabled(imageProcessor, false);
//        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        // Do your paths here.
        telemetry.addData("Identified", imageProcessor.getSelection());
        telemetry.update();
    }
}
