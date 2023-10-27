package org.firstinspires.ftc.teamcode.processors;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Auto EOCV-SIM", group = "Sim")
//@Disabled
public class AutoTestEOCV extends OpMode {
    private ImageProcessor imageProcessor;
    private VisionPortal.Builder visionPortalBuilder;
    private VisionPortal visionPortal;
    private ImageProcessor.Selected selectedSpike;

    @Override
    public void init() {
        imageProcessor = new ImageProcessor(telemetry);
        visionPortalBuilder = new VisionPortal.Builder();
        visionPortal = visionPortalBuilder.enableLiveView(true).
                addProcessor(imageProcessor).
                setCamera(hardwareMap.get(WebcamName.class, "0")).
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
