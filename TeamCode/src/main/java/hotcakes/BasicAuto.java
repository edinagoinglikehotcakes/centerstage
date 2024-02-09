package hotcakes;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

import hotcakes.processors.ImageProcessor;

@Autonomous(name = "auto", group = "Competition")
public class BasicAuto extends OpMode {
    MecanumDrive drive;
    private ImageProcessor imageProcessor;
    private RobotHardware robotHardware;
    private MotorControl motorControl;
    private VisionPortal.Builder visionPortalBuilder;
    private VisionPortal visionPortal;
    private ImageProcessor.Selected selectedSpike;
    private Telemetry.Item teleSelected;
    private ElapsedTime runTime;
    private GamepadEx gamepad;
    private int delaySeconds;
    AutonomousConfiguration autonomousConfiguration = new AutonomousConfiguration();

    private enum AutoState {
        SPIKE,
        DELAY,
        BACKSTAGE,
        PARK,
        DONE,
    }

    AutoState currentAutoState = AutoState.SPIKE;

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
        robotHardware = new RobotHardware(this);
//        robotHardware.init();
        motorControl = new MotorControl(robotHardware);

        runTime = new ElapsedTime();
        teleSelected = telemetry.addData("Selected", teleSelected);
        autonomousConfiguration.init(gamepad, this.telemetry, hardwareMap.appContext);
//        motorControl.moveGripper(MotorControl.GripperState.CLOSE, MotorControl.GripperSelection.BOTH);
    }

    @Override
    public void init_loop() {
        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }

        // Use auto white balance
        if (visionPortal.getCameraControl(WhiteBalanceControl.class).getMode() != WhiteBalanceControl.Mode.AUTO) {
            visionPortal.getCameraControl(WhiteBalanceControl.class).setMode(WhiteBalanceControl.Mode.AUTO);
        }

        // Keep checking the camera
        selectedSpike = imageProcessor.getSelection();
        teleSelected.setValue(selectedSpike);
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
            visionPortal.stopStreaming();
            while (runTime.seconds() < 2) {
            }
            requestOpModeStop();
        }

        // Save resources
        visionPortal.stopStreaming();
        // Get drive started
        drive = new MecanumDrive(hardwareMap, getDriveStartPose());
        delaySeconds = autonomousConfiguration.getDelayStartSeconds();
        runTime.reset();
    }

    @Override
    public void loop() {
        switch (currentAutoState) {
            case SPIKE:
                if (autonomousConfiguration.getPlaceTeamArtOnSpike() == AutonomousOptions.PlaceTeamArtOnSpike.Yes) {

                    placePropOnSpike();
                }

                currentAutoState = AutoState.BACKSTAGE;
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
    }

    @Override
    public void stop() {
    }

    private void placePropOnSpike() {
        {
            AutonomousOptions.AllianceColor allianceColor = autonomousConfiguration.getAlliance();
            AutonomousOptions.StartPosition startPosition = autonomousConfiguration.getStartPosition();
            if (allianceColor == AutonomousOptions.AllianceColor.Red) {
                if (startPosition == AutonomousOptions.StartPosition.Left) {
                    switch (selectedSpike) {
                        case LEFT:
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .splineToConstantHeading(new Vector2d(-46, -50), Math.toRadians(90))
//TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                            .lineToY(-58)
                                            .build());
                            break;
                        case MIDDLE:
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
//                                            .lineToY(-45)
////TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
//                                            .lineToY(-58)
                                            .lineToY(-24)
                                            .build());
                            break;
                        case RIGHT:
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .lineToY(-45)
                                            .turn(Math.toRadians(-90))
                                            .splineToConstantHeading(new Vector2d(-33, -45), Math.toRadians(0))
//TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                            .lineToX(-40)
                                            .strafeToConstantHeading(new Vector2d(-35, -58))
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
                                            .lineToY(-45)
                                            .turn(Math.toRadians(90))
                                            .splineToConstantHeading(new Vector2d(9, -45), Math.toRadians(180))
//TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                            .lineToX(23)
                                            .strafeToLinearHeading(new Vector2d(23, -58), Math.toRadians(0))
                                            .build());
                            break;
                        case MIDDLE:
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .lineToY(-45)
//TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                            .lineToY(-58)
                                            .build());
                            break;
                        case RIGHT:
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .splineToConstantHeading(new Vector2d(29, -50), Math.toRadians(-120))
//TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                            .lineToY(-58)
                                            .build());

                            break;
                        case NONE:
                        default:
                    }
                }
            } else {
                if (allianceColor == AutonomousOptions.AllianceColor.Blue) {
                    if (startPosition == AutonomousOptions.StartPosition.Left) {
                        switch (selectedSpike) {
                            case LEFT:
                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .lineToY(40)
                                                .turn(Math.toRadians(90))
                                                .lineToX(-24)
                                                .lineToX(-36)
                                                .build());
                                break;
                            case MIDDLE:
                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .lineToY(34)
                                                .lineToY(45)
                                                .build());
                                break;
                            case RIGHT:
                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .lineToY(40)
                                                .turn(Math.toRadians(-45))
                                                .splineToConstantHeading(new Vector2d(9, 33), Math.toRadians(45))
                                                .splineToConstantHeading(new Vector2d(14, 38), Math.toRadians(45))
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
                                                .lineToY(47.5)
                                                .turn(Math.toRadians(90))
                                                .lineToX(-24)
                                                .lineToX(-36)
                                                .build());
                                break;
                            case MIDDLE:
                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .lineToY(30)
                                                .lineToY(60)
                                                .build());
                                break;
                            case RIGHT:
                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .splineTo(new Vector2d(-46,47.5), Math.toRadians(270))
                                                .lineToY(60)
                                                .build());
                                break;
                            case NONE:
                            default:
                        }
                    }
                }
            }
        }
    }

    private void placePixelInBackstage() {
        {
            AutonomousOptions.AllianceColor allianceColor = autonomousConfiguration.getAlliance();
            AutonomousOptions.StartPosition startPosition = autonomousConfiguration.getStartPosition();
            if (allianceColor == AutonomousOptions.AllianceColor.Red) {
                if (startPosition == AutonomousOptions.StartPosition.Left) {
                    switch (selectedSpike) {
                        case LEFT:
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .turn(Math.toRadians(-45))
                                            .lineToX(48)
                                            .build());
                            break;
                        case MIDDLE:
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .turn(Math.toRadians(-45))
                                            .lineToX(48)
                                            .build());
                            break;
                        case RIGHT:
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .turn(Math.toRadians(-45))
                                            .lineToX(48)
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
                                            .turn(Math.toRadians(-45))
                                            .lineToX(48)
                                            .build());
                            break;
                        case MIDDLE:
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .turn(Math.toRadians(-45))
                                            .lineToX(48)
                                            .build());
                            break;
                        case RIGHT:
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .turn(Math.toRadians(-45))
                                            .lineToX(48)
                                            .build());

                            break;
                        case NONE:
                        default:
                    }
                }
            } else {
                if (allianceColor == AutonomousOptions.AllianceColor.Blue) {
                    if (startPosition == AutonomousOptions.StartPosition.Left) {
                        switch (selectedSpike) {
                            case LEFT:
                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .turn(Math.toRadians(-90))
                                                .lineToY(60)
                                                .turn(Math.toRadians(90))
                                                .lineToX(47)
                                                .turn(Math.toRadians(-90))
                                                .lineToY(36)
                                                .turn(Math.toRadians(90))
                                                .build());
                                break;
                            case MIDDLE:
                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .lineToY(58)
                                                .lineToX(58)
                                                .build());
                                break;
                            case RIGHT:
                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .splineToConstantHeading(new Vector2d(36, 58), Math.toRadians(270))
                                                .lineToX(58)
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
                                                .turn(Math.toRadians(-90))
                                                .lineToY(60)
                                                .turn(Math.toRadians(90))
                                                .lineToX(47)
                                                .turn(Math.toRadians(-90))
                                                .lineToY(36)
                                                .turn(Math.toRadians(90))
                                                .build());
                                break;
                            case MIDDLE:
                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .turn(Math.toRadians(90))
                                                .lineToX(47)
                                                .turn(Math.toRadians(-90))
                                                .lineToY(35)
                                                .turn(Math.toRadians(90))
                                                .build());
                                break;
                            case RIGHT:
                                Actions.runBlocking(
                                        drive.actionBuilder(drive.pose)
                                                .turn(Math.toRadians(90))
                                                .lineToX(47)
                                                .turn(Math.toRadians(-90))
                                                .lineToY(35)
                                                .turn(Math.toRadians(90))
                                                .build());
                                break;
                            case NONE:
                            default:
                        }
                    }
                }
            }
        }
    }

    // Set the start pose based on the starting position.
    private Pose2d getDriveStartPose() {
        AutonomousOptions.StartPosition startPosition = autonomousConfiguration.getStartPosition();
        double startPositionX;
        double startPositionY;
        double startHeading;
        if (autonomousConfiguration.getAlliance() == AutonomousOptions.AllianceColor.Red) {
            startHeading = Math.toRadians(90);
            startPositionY = -64f;
            if (startPosition == AutonomousOptions.StartPosition.Left) {
                startPositionX = -36f;
            } else {
                startPositionX = 12f;
            }
        } else {
            // Blue Alliance
            startHeading = Math.toRadians(-90);
            startPositionY = 64f;
            if (startPosition == AutonomousOptions.StartPosition.Left) {
                startPositionX = 12f;
            } else {
                startPositionX = -36f;
            }
        }

        return new Pose2d(startPositionX, startPositionY, startHeading);
    }

    private Action buildSpikeActions() {
        AutonomousOptions.AllianceColor allianceColor = autonomousConfiguration.getAlliance();
        AutonomousOptions.StartPosition startPosition = autonomousConfiguration.getStartPosition();
        double heading = allianceColor == AutonomousOptions.AllianceColor.Red ? -90 : 90;
        double positionXLeftSpike;
        double positionXRightSpike;
        double positionYLeftRightSpike;
        double positionYMiddleSpike;
        double positionYAdjustStart;
        double positionYEnd;

        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(drive.pose);
        positionYMiddleSpike = allianceColor == AutonomousOptions.AllianceColor.Blue ? 30 : -30;
        positionYLeftRightSpike = allianceColor == AutonomousOptions.AllianceColor.Blue ? 36 : -36;
        positionYAdjustStart = allianceColor == AutonomousOptions.AllianceColor.Blue ? 40 : -40;
        positionYEnd = allianceColor == AutonomousOptions.AllianceColor.Blue ? 60 : -60;
        if (startPosition == AutonomousOptions.StartPosition.Left) {
            positionXLeftSpike = allianceColor == AutonomousOptions.AllianceColor.Blue ? 24 : -46;
            positionXRightSpike = allianceColor == AutonomousOptions.AllianceColor.Blue ? 1 : -30;
        } else {
            // Start Right
            positionXLeftSpike = allianceColor == AutonomousOptions.AllianceColor.Blue ? 1 : 24;
            positionXRightSpike = allianceColor == AutonomousOptions.AllianceColor.Blue ? -46 : 30;
        }

        switch (selectedSpike) {
            case LEFT:
                if (startPosition == AutonomousOptions.StartPosition.Left) {
                    trajectoryActionBuilder
                            .lineToY(positionYLeftRightSpike);
                } else {
                    // Start Right
                    trajectoryActionBuilder
                            .lineToY(positionYAdjustStart)
                            .splineTo(new Vector2d(positionXLeftSpike, positionYLeftRightSpike), Math.toRadians(heading));
                }
                break;
            case MIDDLE:
                trajectoryActionBuilder
                        .lineToY(positionYMiddleSpike)
                        .lineToY(positionYEnd);
                break;
            case RIGHT:
                if (startPosition == AutonomousOptions.StartPosition.Left) {
                    trajectoryActionBuilder
                            .lineToY(positionYAdjustStart)
                            .splineTo(new Vector2d(positionXRightSpike, positionYLeftRightSpike), Math.toRadians(heading));
                } else {
                    // Start Right
                    trajectoryActionBuilder
                            .lineToY(positionYLeftRightSpike)
                            .splineTo(new Vector2d(positionXLeftSpike, positionYLeftRightSpike), Math.toRadians(heading));
                }
                break;
        }
        return trajectoryActionBuilder
                .build();
    }

    private void placePixelOnBackDrop() {
    }

    private void parkInBackstage() {
    }
}

