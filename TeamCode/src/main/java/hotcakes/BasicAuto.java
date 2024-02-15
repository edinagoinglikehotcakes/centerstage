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
    AutonomousConfiguration autonomousConfiguration = new AutonomousConfiguration();
    Action SpikeActions;
    Action BackdropActions;

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
                                    .splineToConstantHeading(new Vector2d(-45, -50), Math.toRadians(90))
//TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                    .stopAndAdd(armExtension.ArmPickup())
                                    .stopAndAdd(gripperAngle.GripperPickup())
                                    .waitSeconds(1)
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(2)
                                    .stopAndAdd(armExtension.ArmRetract())
                                    .stopAndAdd(gripperAngle.GripperRetract())
                                    .lineToYSplineHeading((-59), Math.toRadians(0))
                                    .build();
                            break;
                        case MIDDLE:
                            SpikeActions = drive.actionBuilder(drive.pose)
                                    .stopAndAdd(armExtension.ArmPickup())
                                    .stopAndAdd(gripperAngle.GripperPickup())
                                    .waitSeconds(1)
                                    .lineToY(-40)
////TODO                                         INSERT ARM EXTENSION, GRIPPER OPEN, AND ARM RETRACTION HERE
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(2)
                                    .stopAndAdd(armExtension.ArmRetract())
                                    .stopAndAdd(gripperAngle.GripperRetract())
                                    .lineToYSplineHeading((-59), Math.toRadians(0))
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
                                    .strafeToSplineHeading(new Vector2d(-40, -59), Math.toRadians(0))
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

                                    .waitSeconds(1)
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(2)
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
                                    .waitSeconds(1)
                                    .lineToY(-40)
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(2)
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
                                    .waitSeconds(1)
                                    .stopAndAdd(gripper.GripperLeftOpen())
                                    .waitSeconds(2)
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

                                        .splineToConstantHeading(new Vector2d(22, 50), Math.toRadians(-90))
                                        .stopAndAdd(armExtension.ArmPickup())
                                        .stopAndAdd(gripperAngle.GripperPickup())
                                        .waitSeconds(1)
                                        .stopAndAdd(gripper.GripperLeftOpen())
                                        .waitSeconds(2)
                                        .stopAndAdd(armExtension.ArmRetract())
                                        .stopAndAdd(gripperAngle.GripperRetract())
                                        .strafeToLinearHeading(new Vector2d(23, 60), Math.toRadians(0))
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
                                        .lineToYSplineHeading((60), Math.toRadians(0))
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
                                        .strafeToSplineHeading(new Vector2d(23, 50), Math.toRadians(0))
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
                                        .strafeToSplineHeading(new Vector2d(-36, 60), Math.toRadians(0))
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
                                        .lineToYSplineHeading((59), Math.toRadians(0))
                                        .build();
                                break;
                            case RIGHT:
                                SpikeActions = drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(new Vector2d(-50, 50), Math.toRadians(-90))
                                        .stopAndAdd(gripperAngle.GripperPickup())
                                        .waitSeconds(1)
                                        .stopAndAdd(gripper.GripperLeftOpen())
                                        .waitSeconds(2)
                                        .stopAndAdd(armExtension.ArmRetract())
                                        .stopAndAdd(gripperAngle.GripperRetract())
                                        .strafeToLinearHeading(new Vector2d(-36, 59), Math.toRadians(0)).stopAndAdd(armExtension.ArmPickup())
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
                                    .lineToX(42)
                                    .splineToConstantHeading(new Vector2d(42, -28), Math.toRadians(0))
//                                    .stopAndAdd(armlift.LiftBackDropAngle())
//                                    .stopAndAdd(armExtension.ArmBackDrop())
//                                    .waitSeconds(3)
//                                    .stopAndAdd(gripperAngle.GripperBackDrop())
//                                    .waitSeconds(1)
//                                    .stopAndAdd(gripper.GripperRightOpen())
//                                    .waitSeconds(0.5)
//                                    .stopAndAdd(armExtension.ArmRetract())
//                                    .stopAndAdd(armlift.liftPickup())
                                    .stopAndAdd(getBackdropActions())
                                    .splineToConstantHeading(new Vector2d(47, -13), Math.toRadians(0))

                                    .build();
                            break;
                        case MIDDLE:
                            BackdropActions = drive.actionBuilder(drive.pose)
                                    .setTangent(0)
                                    .lineToX(42)
                                    .splineToConstantHeading(new Vector2d(42, -35), Math.toRadians(0))
                                    .stopAndAdd(getBackdropActions())

                                    .splineToConstantHeading(new Vector2d(47, -13), Math.toRadians(0))
                                    .build();
                            break;
                        case RIGHT:
                            BackdropActions = drive.actionBuilder(drive.pose)
                                    .setTangent(0)
                                    .lineToX(42)
                                    .splineToConstantHeading(new Vector2d(42, -42), Math.toRadians(0))
                                    .stopAndAdd(getBackdropActions())

                                    .splineToConstantHeading(new Vector2d(47, -13), Math.toRadians(0))
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
                                    .lineToX(42)
                                    .splineToConstantHeading(new Vector2d(42, -28), Math.toRadians(0))
                                    .stopAndAdd(getBackdropActions())

                                    .splineToConstantHeading(new Vector2d(47, -13), Math.toRadians(0))
                                    .build();
                            break;
                        case MIDDLE:
                            BackdropActions = drive.actionBuilder(drive.pose)

                                    .setTangent(0)
                                    .lineToX(42)
                                    .splineToConstantHeading(new Vector2d(42, -35), Math.toRadians(0))
                                    .stopAndAdd(getBackdropActions())

                                    .splineToConstantHeading(new Vector2d(47, -13), Math.toRadians(0))
                                    .build();
                            break;
                        case RIGHT:
                            BackdropActions = drive.actionBuilder(drive.pose)

                                    .setTangent(0)
                                    .lineToX(42)
                                    .splineToConstantHeading(new Vector2d(42, -42), Math.toRadians(0))
                                    .stopAndAdd(getBackdropActions())

                                    .splineToConstantHeading(new Vector2d(47, -13), Math.toRadians(0))
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

                                        .setTangent(0)
                                        .lineToX(42)
                                        .splineToConstantHeading(new Vector2d(42, 42), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())

                                        .splineToConstantHeading(new Vector2d(47, 12), Math.toRadians(0))
                                        .build();
                                break;
                            case MIDDLE:
                                BackdropActions = drive.actionBuilder(drive.pose)

                                        .setTangent(0)
                                        .lineToX(42)
                                        .splineToConstantHeading(new Vector2d(42, 35), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())

                                        .splineToConstantHeading(new Vector2d(47, 12), Math.toRadians(0))
                                        .build();
                                break;
                            case RIGHT:
                                BackdropActions = drive.actionBuilder(drive.pose)

                                        .setTangent(0)
                                        .lineToX(42)
                                        .splineToConstantHeading(new Vector2d(42, 28), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())

                                        .splineToConstantHeading(new Vector2d(47, 12), Math.toRadians(0))
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
                                        .lineToX(42)
                                        .splineToConstantHeading(new Vector2d(42, 42), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())

                                        .splineToConstantHeading(new Vector2d(47, 12), Math.toRadians(0))
                                        .build();
                                break;
                            case MIDDLE:
                                BackdropActions = drive.actionBuilder(drive.pose)

                                        .setTangent(0)
                                        .lineToX(42)
                                        .splineToConstantHeading(new Vector2d(42, 35), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())

                                        .splineToConstantHeading(new Vector2d(47, 12), Math.toRadians(0))
                                        .build();
                                break;
                            case RIGHT:
                                BackdropActions = drive.actionBuilder(drive.pose)

                                        .setTangent(0)
                                        .lineToX(42)
                                        .splineToConstantHeading(new Vector2d(42, 28), Math.toRadians(0))
                                        .stopAndAdd(getBackdropActions())

                                        .splineToConstantHeading(new Vector2d(47, 12), Math.toRadians(0))
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
                .waitSeconds(3)
                .stopAndAdd(gripperAngle.GripperBackDrop())
                .waitSeconds(1)
                .stopAndAdd(gripper.GripperRightOpen())
                .waitSeconds(0.5)
                .stopAndAdd(armExtension.ArmRetract())
                .stopAndAdd(armlift.liftPickup());
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

    private void placePixelOnBackDrop() {
    }

    private void parkInBackstage() {
    }
}

