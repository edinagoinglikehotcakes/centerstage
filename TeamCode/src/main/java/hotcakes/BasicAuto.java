package hotcakes;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
    private GamepadEx gamepad;
    private double startPositionX = 36f;
    private double startPositionY = -60f;
    private double startHeading = Math.toRadians(90);
    AutonomousConfiguration autonomousConfiguration = new AutonomousConfiguration();

    @Override
    public void init() {
        gamepad = new GamepadEx(gamepad1);
        autonomousConfiguration.init(gamepad, this.telemetry, hardwareMap.appContext);
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
        // Keep checking the camera
        selectedSpike = imageProcessor.getSelection();
        telemetry.addData("Spike Identified", selectedSpike);
        telemetry.update();
        // Get the menu options
        autonomousConfiguration.init_loop();
    }

    @Override
    public void start() {
        // Make sure the required menu options are set.
        if (!autonomousConfiguration.getReadyToStart()) {
            telemetry.addData("Alert", "Not ready to start!");
            telemetry.speak("Not ready to start!");
            runTime.reset();
            while (runTime.seconds() < 2) {
            }
            requestOpModeStop();
        }

        // Save resources
        visionPortal.stopStreaming();
        //Set drive to the starting pose.
        setDriveStartPose();
        int delaySeconds = autonomousConfiguration.getDelayStartSeconds();
        runTime.reset();
        if (delaySeconds > 0) {
            while (runTime.seconds() <= delaySeconds) {

            }
        }

        runTime.reset();
    }

    @Override
    public void loop() {
        if (autonomousConfiguration.getPlaceTeamArtOnSpike() == AutonomousOptions.PlaceTeamArtOnSpike.Yes) {
            placePropOnSpike();
        }

        drive.updatePoseEstimate();
        telemetry.addData("runTime", runTime.seconds());
        telemetry.update();
    }

    private void placePropOnSpike() {
        AutonomousOptions.AllianceColor allianceColor = autonomousConfiguration.getAlliance();
        switch (selectedSpike) {
            case LEFT:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(-46, -30), Math.toRadians(90))
                                .lineToX(-5)
                                .turn(Math.toRadians(-90))
                                .lineToX(90)
                                .lineToX(-5)
                                .splineTo(new Vector2d(60, -60), Math.toRadians(90))
                                .build());
                break;
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
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(-24, -30), Math.toRadians(90))
                                .lineToX(-5)
                                .turn(Math.toRadians(-90))
                                .lineToX(70)
                                .lineToX(-5)
                                .splineTo(new Vector2d(60, -60), Math.toRadians(90))
                                .build());
                break;
            case NONE:
            default:
        }
    }

    private void setDriveStartPose() {
        AutonomousOptions.StartPosition startPosition = autonomousConfiguration.getStartPosition();
        if (autonomousConfiguration.getAlliance() == AutonomousOptions.AllianceColor.Red) {
            startHeading = Math.toRadians(90);
            startPositionY = -60f;
            if (startPosition == AutonomousOptions.StartPosition.Left) {
                startPositionX = -36f;
            } else {
                startPositionX = -12f;
            }
        } else {
            startHeading = Math.toRadians(-90);
            startPositionY = 60f;
            if (startPosition == AutonomousOptions.StartPosition.Left) {
                startPositionX = -12f;
            } else {
                startPositionX = -36f;
            }
        }
        drive = new MecanumDrive(hardwareMap, new Pose2d(startPositionX, startPositionY, startHeading));
    }

    private void placePixelInBackstage() {

    }

    private void placePixelOnBackDrop() {

    }

    private void parkInBackstage() {

    }
}

