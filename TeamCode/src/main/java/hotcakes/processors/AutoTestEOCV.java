package hotcakes.processors;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Auto EOCV-SIM", group = "Sim")
//@Disabled
public class AutoTestEOCV extends OpMode {
    private ImageProcessor imageProcessor;
    private VisionPortal.Builder visionPortalBuilder;
    private VisionPortal visionPortal;
    private ImageProcessor.Selected selectedSpike;
    private WhiteBalanceControl whiteBalanceControl;
    private MecanumDrive mecanumDrive;
    private GamepadEx gamepadEx;

    @Override
    public void init() {
        gamepadEx = new GamepadEx(gamepad1);
//        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
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
        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            return;
        } else {
            whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        }

        if (whiteBalanceControl.getMode() != WhiteBalanceControl.Mode.AUTO) {
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.AUTO);
        }

        telemetry.addData("Identified", imageProcessor.getSelection());
        telemetry.addData("Left", "%5.2f", imageProcessor.satRectLeft);
        telemetry.addData("Middle", "%5.2f", imageProcessor.satRectMiddle);
        telemetry.addData("Right", "%5.2f", imageProcessor.satRectRight);
    }

    @Override
    public void start() {
//        Actions.runBlocking(
//                mecanumDrive.actionBuilder(mecanumDrive.pose)
////                        .splineTo(new Vector2d(30,30),Math.PI /2)
////                        .splineTo(new Vector2d(60,0),Math.PI)
//                        .turn(Math.toRadians(90))
//                        .build());
        selectedSpike = imageProcessor.getSelection();
        telemetry.addData("Start Identified", selectedSpike);
        // Save resources
//        visionPortal.setProcessorEnabled(imageProcessor, false);
//        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        gamepadEx.readButtons();
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.X)) {
            visionPortal.saveNextFrameRaw("cameraframe");
        }
        // Do your paths here.
        telemetry.addData("Identified", imageProcessor.getSelection());
    }
}
