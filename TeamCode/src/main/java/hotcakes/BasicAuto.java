package hotcakes;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    private ArmExtension armExtension;
    private ArmLift armlift;
    private Gripper gripper;
    private GripperAngle gripperAngle;
//    private BackdropAprilTags backdropAprilTags = null;
    AutonomousConfiguration autonomousConfiguration = new AutonomousConfiguration();
    Action SpikeActions;
    Action BackdropActions;
//    Action AprilTag;

    private enum AutoState {
        SPIKE,
        DELAY,
        BACKSTAGE,
        PARK,
        DONE,
    }

//    private Action AprilTag() {
//        AutonomousOptions.StartPosition startPosition = autonomousConfiguration.getStartPosition();
//        backdropAprilTags = new BackdropAprilTags(robotHardware.myOpMode.hardwareMap);
//        backdropAprilTags.init();
//        AprilTagDetection detectedTag = backdropAprilTags.detectTags();
//        if (startPosition == AutonomousOptions.StartPosition.Left) {
//            if (selectedSpike == AutonomousOptions. {
//                if (detectedTag == backdropAprilTags.getDetection()) {
//                }
//            }
//        }
//        backdropAprilTags.disableTagProcessing();
//        // Set the launch angle
//
//    }

    AutoState currentAutoState = AutoState.SPIKE;

    @Override
    public void init() {
//        robotHardware = new RobotHardware(this);
//        robotHardware.init();
//        motorControl = new MotorControl(robotHardware);
        armExtension = new ArmExtension(this);
        gripper = new Gripper(this);
        armlift = new ArmLift(this);
        gripperAngle = new GripperAngle(this);
        gamepad = new GamepadEx(gamepad1);
        imageProcessor = new ImageProcessor(telemetry);
        visionPortalBuilder = new VisionPortal.Builder();
        visionPortal = visionPortalBuilder.enableLiveView(true).
                addProcessor(imageProcessor).
                setCamera(hardwareMap.get(WebcamName.class, "webcam1")).
                setCameraResolution(new Size(640, 480)).
                build();
//        robotHardware.init();

        runTime = new ElapsedTime();
        teleSelected = telemetry.addData("Selected", teleSelected);
        autonomousConfiguration.init(gamepad, this.telemetry, hardwareMap.appContext);
//        motorControl.moveGripper(MotorControl.GripperState.CLOSE, MotorControl.GripperSelection.BOTH);
        Actions.runBlocking(gripper.GripperLeftClose());
        Actions.runBlocking(gripper.GripperRightClose());
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
                    Actions.runBlocking(SpikeActions);

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
                    Actions.runBlocking(BackdropActions);
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
//        drive.updatePoseEstimate();
        telemetry.addData("Selected", selectedSpike);
        telemetry.addData("State", currentAutoState);
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
                            SpikeActions = drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(new Vector2d(-46.5, -50), Math.toRadians(90))
//TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                    .stopAndAdd(armExtension.ArmPickup())
                                    .stopAndAdd(gripperAngle.GripperPickup())
                                    .waitSeconds(1)
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(2)
                                    .stopAndAdd(armExtension.ArmRetract())
                                    .stopAndAdd(gripperAngle.GripperRetract())
                                    .lineToY(-59)
                                    .turn(Math.toRadians(-90))

                                    .build();
                            break;
                        case MIDDLE:
                            SpikeActions = drive.actionBuilder(drive.pose)
                                    .stopAndAdd(armExtension.ArmPickup())
                                    .stopAndAdd(gripperAngle.GripperPickup())
                                    .waitSeconds(1)
                                    .lineToY(-38)
////TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(2)
                                    .stopAndAdd(armExtension.ArmRetract())
                                    .stopAndAdd(gripperAngle.GripperRetract())
                                    .lineToY(-59)
                                    .turn(Math.toRadians(-90))
                                    .build();
                            break;
                        case RIGHT:
                            SpikeActions = drive.actionBuilder(drive.pose)
//                                    TODO CHANGE THE CODE TO GET TO SPIKE
                                    .stopAndAdd(armExtension.ArmPickup())
                                    .stopAndAdd(gripperAngle.GripperPickup())
                                    .lineToY(-43)
                                    .turn(Math.toRadians(-40))
//TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                    .waitSeconds(1)
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(2)
                                    .stopAndAdd(armExtension.ArmRetract())
                                    .stopAndAdd(gripperAngle.GripperRetract())
                                    .turn(Math.toRadians(40))
                                    .lineToY(-59)
                                    .turn(Math.toRadians(-90))
//                                    .strafeToConstantHeading(new Vector2d(-35, -58))
                                    .build();

                            break;
                        case NONE:
                        default:
                    }
                } else {
                    switch (selectedSpike) {
                        case LEFT:
                            SpikeActions = drive.actionBuilder(drive.pose)
                                    .stopAndAdd(armExtension.ArmPickup())
                                    .stopAndAdd(gripperAngle.GripperPickup())
                                    .lineToY(-43)
                                    .turn(Math.toRadians(30))
//TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE

                                    .waitSeconds(0.7)
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(0.3)
                                    .stopAndAdd(armExtension.ArmRetract())
                                    .stopAndAdd(gripperAngle.GripperRetract())
                                    .turn(Math.toRadians(-30))
                                    .strafeToSplineHeading(new Vector2d(23, -59), Math.toRadians(0))
                                    .build();
                            break;
                        case MIDDLE:
                            SpikeActions = drive.actionBuilder(drive.pose)
                                    .stopAndAdd(armExtension.ArmPickup())
                                    .stopAndAdd(gripperAngle.GripperPickup())
                                    .waitSeconds(0.7)
                                    .lineToY(-39)
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(0.3)
                                    .stopAndAdd(armExtension.ArmRetract())
                                    .stopAndAdd(gripperAngle.GripperRetract())
                                    .lineToYSplineHeading((-58), Math.toRadians(0))
                                    .build();
                            break;
                        case RIGHT:
                            SpikeActions = drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(new Vector2d(27, -50), Math.toRadians(90))
//TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                    .stopAndAdd(armExtension.ArmPickup())
                                    .stopAndAdd(gripperAngle.GripperPickup())
                                    .waitSeconds(0.7)
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(0.3)
                                    .stopAndAdd(armExtension.ArmRetract())
                                    .stopAndAdd(gripperAngle.GripperRetract())
                                    .strafeToSplineHeading(new Vector2d(23, -59), Math.toRadians(0))
                                    .build();

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
                                SpikeActions = drive.actionBuilder(drive.pose)
                                        .stopAndAdd(armExtension.ArmPickup())
                                        .stopAndAdd(gripperAngle.GripperPickup())
                                        .splineToConstantHeading(new Vector2d(22, 50), Math.toRadians(-90))
                                        .waitSeconds(1)
                                        .stopAndAdd(gripper.GripperLeftOpen())
                                        .waitSeconds(2)
                                        .stopAndAdd(armExtension.ArmRetract())
                                        .stopAndAdd(gripperAngle.GripperRetract())
                                        .lineToY(58)
                                        .turn(Math.toRadians(90))
                                        .build();
                                break;
                            case MIDDLE:
                                SpikeActions = drive.actionBuilder(drive.pose)
                                        .stopAndAdd(armExtension.ArmPickup())
                                        .stopAndAdd(gripperAngle.GripperPickup())
                                        .lineToY(40)
                                        .waitSeconds(1)
                                        .stopAndAdd(gripper.GripperLeftOpen())
                                        .waitSeconds(2)
                                        .stopAndAdd(armExtension.ArmRetract())
                                        .stopAndAdd(gripperAngle.GripperRetract())
                                        .lineToY(58)
                                        .turn(Math.toRadians(90))
                                        .build();
                                break;
                            case RIGHT:
                                SpikeActions = drive.actionBuilder(drive.pose)
                                        .stopAndAdd(armExtension.ArmPickup())
                                        .stopAndAdd(gripperAngle.GripperPickup())
                                        .lineToY(43)
                                        .turn(Math.toRadians(-40))
                                        .waitSeconds(1)
                                        .stopAndAdd(gripper.GripperLeftOpen())
                                        .waitSeconds(2)
                                        .stopAndAdd(armExtension.ArmRetract())
                                        .stopAndAdd(gripperAngle.GripperRetract())
                                        .waitSeconds(2)
                                        .turn(Math.toRadians(40))
                                        .lineToY(58)
                                        .turn(Math.toRadians(90))
                                        .build();

                                break;
                            case NONE:
                            default:
                        }
                    } else {
                        switch (selectedSpike) {
                            case LEFT:
                                SpikeActions = drive.actionBuilder(drive.pose)
                                        .stopAndAdd(armExtension.ArmPickup())
                                        .stopAndAdd(gripperAngle.GripperPickup())
                                        .lineToY(44)
                                        .turn(Math.toRadians(30))
                                        .waitSeconds(1)
                                        .stopAndAdd(gripper.GripperLeftOpen())
                                        .waitSeconds(2)
                                        .stopAndAdd(armExtension.ArmRetract())
                                        .stopAndAdd(gripperAngle.GripperRetract())
                                        .lineToY(57)
                                        .turn(Math.toRadians(90))
                                        .build();
                                break;
                            case MIDDLE:
                                SpikeActions = drive.actionBuilder(drive.pose)
                                        .stopAndAdd(armExtension.ArmPickup())
                                        .stopAndAdd(gripperAngle.GripperPickup())
                                        .lineToY(40)
                                        .waitSeconds(1)
                                        .stopAndAdd(gripper.GripperLeftOpen())
                                        .waitSeconds(2)
                                        .stopAndAdd(armExtension.ArmRetract())
                                        .stopAndAdd(gripperAngle.GripperRetract())
                                        .lineToY(57)
                                        .turn(Math.toRadians(90))
                                        .build();
                                break;
                            case RIGHT:
                                SpikeActions = drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(new Vector2d(-50, 50), Math.toRadians(-90))
                                        .stopAndAdd(armExtension.ArmPickup())
                                        .stopAndAdd(gripperAngle.GripperPickup())
                                        .waitSeconds(1)
                                        .stopAndAdd(gripper.GripperLeftOpen())
                                        .waitSeconds(2)
                                        .stopAndAdd(armExtension.ArmRetract())
                                        .stopAndAdd(gripperAngle.GripperRetract())
                                        .lineToY(57)
                                        .turn(Math.toRadians(90))
                                        .build();
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
                            BackdropActions = drive.actionBuilder(drive.pose)
                                    .setTangent(0)
                                    .lineToX(30)
                                    .splineToConstantHeading(new Vector2d(58.5, -18.7), Math.toRadians(0))
                                    .stopAndAdd(getBackdropActions())
                                    .setTangent(0)
                                    .lineToX(45)
                                    .splineToConstantHeading(new Vector2d(50, 0), Math.toRadians(0))
                                    .setTangent(0)
                                    .lineToX(63)

                                    .build();
                            break;
                        case MIDDLE:
                            BackdropActions = drive.actionBuilder(drive.pose)
                                    .setTangent(0)
                                    .lineToX(30)
                                    .splineToConstantHeading(new Vector2d(58.5, -29), Math.toRadians(0))
                                    .stopAndAdd(getBackdropActions())
                                    .setTangent(0)
                                    .lineToX(45)
                                    .splineToConstantHeading(new Vector2d(50, 0), Math.toRadians(0))
                                    .setTangent(0)
                                    .lineToX(63)
                                    .build();
                            break;
                        case RIGHT:
                            BackdropActions = drive.actionBuilder(drive.pose)
                                    .setTangent(0)
                                    .lineToX(30)
                                    .splineToConstantHeading(new Vector2d(58.5, -37.6), Math.toRadians(10))
                                    .stopAndAdd(getBackdropActions())
                                    .setTangent(0)
                                    .lineToX(45)

                                    .splineToConstantHeading(new Vector2d(53, 0), Math.toRadians(0))
                                    .setTangent(0)
                                    .lineToX(63)
                                    .build();

                            break;
                        case NONE:
                        default:
                    }
                } else {
                    switch (selectedSpike) {
                        case LEFT:
                            BackdropActions = drive.actionBuilder(drive.pose)

                                    .setTangent(0)
                                    .lineToX(30)
                                    .splineToConstantHeading(new Vector2d(50, -20), Math.toRadians(0))
                                    .stopAndAdd(getBackdropActions())
                                    .setTangent(0)
                                    .lineToX(45)
                                    .splineToConstantHeading(new Vector2d(50, 0), Math.toRadians(0))
//                                    .splineToConstantHeading(new Vector2d(50, 57), Math.toRadians(0))
                                    .setTangent(0)
                                    .lineToX(63)
                                    .build();
                            break;
                        case MIDDLE:
                            BackdropActions = drive.actionBuilder(drive.pose)

                                    .setTangent(0)
                                    .lineToX(30)
                                    .splineToConstantHeading(new Vector2d(48.5, -27.5), Math.toRadians(0))
                                    .stopAndAdd(getBackdropActions())
                                    .setTangent(0)
                                    .lineToX(45)
                                    .splineToConstantHeading(new Vector2d(50, 0), Math.toRadians(0))
//                                    .splineToConstantHeading(new Vector2d(50, 57), Math.toRadians(0))
                                    .setTangent(0)
                                    .lineToX(63)
                                    .build();
                            break;
                        case RIGHT:
                            BackdropActions = drive.actionBuilder(drive.pose)

                                    .setTangent(0)
                                    .lineToX(30)
                                    .splineToConstantHeading(new Vector2d(52.5, -36), Math.toRadians(10))
                                    .stopAndAdd(getBackdropActions())
                                    .setTangent(0)
                                    .lineToX(45)
                                    .splineToConstantHeading(new Vector2d(50, -2), Math.toRadians(0))
//                                    .splineToConstantHeading(new Vector2d(50, 57), Math.toRadians(0))
                                    .setTangent(0)
                                    .lineToX(63)
                                    .build();

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
                                BackdropActions = drive.actionBuilder(drive.pose)
                                        .waitSeconds(5)
                                        .setTangent(0)
                                        .lineToX(30)
                                        .splineToConstantHeading(new Vector2d(52, 42), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())
                                        .setTangent(0)
                                        .lineToX(45)
                                        .splineToConstantHeading(new Vector2d(59, 0), Math.toRadians(0))
                                        .build();
                                break;
                            case MIDDLE:
                                BackdropActions = drive.actionBuilder(drive.pose)
                                        .waitSeconds(5)
                                        .setTangent(0)
                                        .lineToX(30)
                                        .splineToConstantHeading(new Vector2d(56, 30), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())
                                        .setTangent(0)
                                        .lineToX(45)
                                        .splineToConstantHeading(new Vector2d(59, 0), Math.toRadians(0))
                                        .build();
                                break;
                            case RIGHT:
                                BackdropActions = drive.actionBuilder(drive.pose)
                                        .waitSeconds(5)
                                        .setTangent(0)
                                        .lineToX(30)
                                        .splineToConstantHeading(new Vector2d(51.5, 21), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())
                                        .setTangent(0)
                                        .lineToX(45)
                                        .splineToConstantHeading(new Vector2d(59, 0), Math.toRadians(0))
                                        .build();

                                break;
                            case NONE:
                            default:
                        }
                    } else {
                        switch (selectedSpike) {
                            case LEFT:
                                BackdropActions = drive.actionBuilder(drive.pose)
//                                        .waitSeconds(2)
                                        .waitSeconds(5)
                                        .setTangent(0)
                                        .lineToX(30)
                                        .splineToConstantHeading(new Vector2d(56, 42), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())
                                        .setTangent(0)
                                        .lineToX(45)
                                        .splineToConstantHeading(new Vector2d(59, 0), Math.toRadians(0))
                                        .build();
                                break;
                            case MIDDLE:
                                BackdropActions = drive.actionBuilder(drive.pose)
//                                        .waitSeconds(2)
                                        .waitSeconds(5)
                                        .setTangent(0)
                                        .lineToX(30)
                                        .splineToConstantHeading(new Vector2d(56, 28.5), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())
                                        .setTangent(0)
                                        .lineToX(45)
                                        .splineToConstantHeading(new Vector2d(59, 0), Math.toRadians(0))
                                        .build();
                                break;
                            case RIGHT:
                                BackdropActions = drive.actionBuilder(drive.pose)
//                                        .waitSeconds(2)
                                        .waitSeconds(5)
                                        .setTangent(0)
                                        .lineToX(30)
                                        .splineToConstantHeading(new Vector2d(56, 22), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())
                                        .setTangent(0)
                                        .lineToX(45)
                                        .splineToConstantHeading(new Vector2d(59, 0), Math.toRadians(0))
                                        .build();
                                break;
                            case NONE:
                            default:
                        }
                    }
                }
            }
        }
    }

    private Action getBackdropActions() {
        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(drive.pose)
                .stopAndAdd(armlift.LiftBackDropAngle())
                .stopAndAdd(armExtension.ArmBackDrop())
               .waitSeconds(1.7)
                .stopAndAdd(gripperAngle.GripperBackDrop())
                .waitSeconds(0.5)
                .stopAndAdd(gripper.GripperRightOpen())
                .waitSeconds(0.3)
                .stopAndAdd(armExtension.ArmRetract())
                .stopAndAdd(armlift.liftPickup())
                .stopAndAdd(gripperAngle.GripperRetract());
        return trajectoryActionBuilder.build();

    }

    public class Backdrop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            armlift.LiftBackDropAngle();
            armExtension.ArmBackDrop();
//                    wait(3)
            gripperAngle.GripperBackDrop();
//                    .waitSeconds(1)
            gripper.GripperRightOpen();
//                    .waitSeconds(0.5)
            armExtension.ArmRetract();
            armlift.liftPickup();
            return false;
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

