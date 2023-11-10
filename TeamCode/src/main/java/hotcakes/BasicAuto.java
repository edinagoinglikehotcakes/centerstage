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

@Autonomous(name = "auto")
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

    private enum AutoState {
        START,
        SPIKE,
        DELAY,
        BACKSTAGE,
        PARK,
        DONE,

    }

    AutoState currentAutoState = AutoState.START;

    @Override
    public void init() {
        gamepad = new GamepadEx(gamepad1);
        imageProcessor = new ImageProcessor(telemetry);
        visionPortalBuilder = new VisionPortal.Builder();
        visionPortal = visionPortalBuilder.enableLiveView(true).
                addProcessor(imageProcessor).
                setCamera(hardwareMap.get(WebcamName.class, "webcam1")).
                setCameraResolution(new Size(640, 480)).
                build();

        runTime = new ElapsedTime();
//        telemetry.addData("init", "");
//        telemetry.update();
        autonomousConfiguration.init(gamepad, this.telemetry, hardwareMap.appContext);
    }

    @Override
    public void init_loop() {
        // Keep checking the camera
        // Get the menu options
        autonomousConfiguration.init_loop();
        selectedSpike = imageProcessor.getSelection();
//        telemetry.addData("Spike Identified", selectedSpike);
//        telemetry.update();
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
        switch (currentAutoState) {
            case START:
                currentAutoState = AutoState.SPIKE;
                break;
            case SPIKE:
                if (autonomousConfiguration.getPlaceTeamArtOnSpike() == AutonomousOptions.PlaceTeamArtOnSpike.Yes) {
                    placePropOnSpike();
                }
                currentAutoState = AutoState.DONE;
                break;
            case DELAY:
                if (autonomousConfiguration.getDelayStartSeconds() > 0) {
                    //TODO Put delay here
                }
                currentAutoState = AutoState.BACKSTAGE;
                break;
            case BACKSTAGE:
                if (autonomousConfiguration.getPlacePixelsInBackstage() == AutonomousOptions.PlacePixelsInBackstage.Yes) {
                    placePixelInBackstage();
                }
                currentAutoState = AutoState.PARK;
                break;
            case PARK:
                currentAutoState = AutoState.DONE;
                break;
            case DONE:
                break;
            default:
                break;

        }
        drive.updatePoseEstimate();
        telemetry.addData("runTime", runTime.seconds());
        telemetry.update();
    }

    private void placePropOnSpike() {
        AutonomousOptions.AllianceColor allianceColor = autonomousConfiguration.getAlliance();
        AutonomousOptions.StartPosition startPosition = autonomousConfiguration.getStartPosition();
        if (allianceColor == AutonomousOptions.AllianceColor.Red) {
            if (startPosition == AutonomousOptions.StartPosition.Left) {
                //TODO THIS IS THE CODE FOR RED LEFT
                switch (selectedSpike) {
                    case LEFT:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineTo(new Vector2d(-46, -40), Math.toRadians(90))
                                        .build());
                        break;
                    case MIDDLE:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .lineToY(-30)
                                        .lineToY(-45)
                                        .turn(Math.toRadians(-90))
                                        .build());
                        break;
                    case RIGHT:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .lineToY(-40) 
                                        .splineTo(new Vector2d(-30, -30), Math.toRadians(90))
                                        .lineToY(-35)
                                        .build());
                        break;
                    case NONE:
                    default:
                }
            } else {
                switch (selectedSpike) {
                    case LEFT:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        //TODO CHANGE NUMBERS FOR RED RIGHT (Ready to test)
                                        .lineToY(-46)
                                        .splineTo(new Vector2d(0, -33), Math.toRadians(90))
                                        .splineTo(new Vector2d(10, -47), Math.toRadians(0))
                                        .lineToY(-57)
                                        .turn(Math.toRadians(-90))
                                        .build());
                        break;
                    case MIDDLE:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        //TODO CHANGE NUMBERS FOR RED RIGHT (Ready to test)
                                        .lineToY(-28)
                                        .lineToY(-45)
                                        .turn(Math.toRadians(-90))
                                        .build());
                        break;
                    case RIGHT:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        //TODO CHANGE NUMBERS FOR RED RIGHT (Ready to test)
                                        .splineTo(new Vector2d(22.5, -34), Math.toRadians(90))
                                        .lineToY(-45)
                                        .build());
                        break;
                    case NONE:
                    default:
                }
            }
        } else {
            if (startPosition == AutonomousOptions.StartPosition.Left) {
                switch (selectedSpike) {
                    case LEFT:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        //TODO Change numbers BLUE LEFT (Ready to test)
                                        .splineTo(new Vector2d(23, 35), Math.toRadians(90))
                                        .lineToY(45)
                                        .build());
                        break;
                    case MIDDLE:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        //TODO Change numbers FOR BLUE LEFT
                                        .lineToY(27)
                                        .lineToY(45)
                                        .turn(Math.toRadians(90))
                                        .build());
                        break;
                    case RIGHT:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        //TODO Change numbers FOR BLUE LEFT
                                        .lineToY(47)
                                        .splineTo(new Vector2d(2, 34), Math.toRadians(90))
                                        .splineTo(new Vector2d(11, 48), Math.toRadians(0))
                                        .lineToY(57)
                                        .build());
                        break;
                    case NONE:
                    default:
                }
            } else {
                switch (selectedSpike) {
                    case LEFT:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        //TODO Change numbers FOR BLUE RIGHT
                                        .lineToY(47)
                                        .splineTo(new Vector2d(-24, 34), Math.toRadians(90))
                                        .splineTo(new Vector2d(-36, 47), Math.toRadians(0))
                                        .lineToY(57)
                                        .build());
                        break;
                    case MIDDLE:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        //TODO Change numbers FOR BLUE RIGHT
                                        .lineToY(25)
                                        .lineToY(45)
                                        .turn(Math.toRadians(90))
                                        .build());
                        break;
                    case RIGHT:
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        //TODO Change numbers FOR BLUE RIGHT
                                        .splineTo(new Vector2d(-46, 35), Math.toRadians(90))
                                        .lineToY(45)
                                        .build());
                        break;
                    case NONE:
                    default:
                }

            }
        }
    }

    private void placePixelInBackstage() {
        switch (selectedSpike) {
            case LEFT:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .turn(Math.toRadians(-90))
                                .lineToY(90)
                                .splineTo(new Vector2d(40, -34), Math.toRadians(45))
                                .build());
                break;
            case MIDDLE:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .setTangent(0)
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
                                .setTangent(0)
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

    private void placePixelOnBackDrop() {

    }

    private void parkInBackstage() {

    }
}

