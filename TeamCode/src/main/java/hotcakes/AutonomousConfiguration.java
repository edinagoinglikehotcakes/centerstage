package hotcakes;

import android.content.Context;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ron on 11/16/2016.
 * Modified: 09/25/2023
 * <p>
 * This class provides configuration for an autonomous opMode.
 * Most games benefit from autonomous opModes that can implement
 * different behavior based on an alliance strategy agreed upon
 * for a specific match.
 * </p>
 * <p>
 * Creating multiple opModes to meet this requirement results in duplicate
 * code and an environment that makes it too easy for a driver to
 * choose the wrong opMode "in the heat of battle."
 * </p>
 * <p>
 * This class is a way to solve these problems.
 * It is designed to used from opMode (iterative) class.
 * The selected options can also be saved to a file, allowing the
 * configuration options to be set before a match and to be available
 * to any op Mode.
 * </p>
 */

public class AutonomousConfiguration {
    private AutonomousOptions autonomousOptions;
    private Context context;
    private boolean readyToStart;
    private boolean savedToFile;
    private Telemetry telemetry;
    private Telemetry.Item teleAlliance;
    private Telemetry.Item teleStartPosition;
    private Telemetry.Item teleParkInBackstage;
    private Telemetry.Item telePlaceTeamArtOnSpike;
    private Telemetry.Item telePlacePixelsInBackstage;
    private Telemetry.Item telePlacePixelsOnBackdrop;
    private Telemetry.Item teleDelayStartSeconds;
    private Telemetry.Item teleReadyToStart;
    private Telemetry.Item teleSavedToFile;

    private GamepadEx gamePad;

    /*
     * Pass in the gamepad and telemetry from your opMode.
     */
    public void init(GamepadEx gamepad, Telemetry telemetry1, Context context) {
        this.context = context;
        ReadWriteAutoOptions readWriteAutoOptions = new ReadWriteAutoOptions(context);
        gamePad = gamepad;
        this.telemetry = telemetry1;
        // See if we saved the options yet. If not, save the defaults.
        autonomousOptions = new AutonomousOptions();
        if (!readWriteAutoOptions.optionsAreSaved()) {
            resetOptions();
            this.SaveOptions();
        } else {
            autonomousOptions = getSaveAutoOptions();
        }

        ShowHelp();
    }

    public AutonomousOptions.AllianceColor getAlliance() {
        return autonomousOptions.getAllianceColor();
    }

    public AutonomousOptions.StartPosition getStartPosition() {
        return autonomousOptions.getStartPosition();
    }

    public AutonomousOptions.ParkInBackstage getParkInBackstage() {
        return autonomousOptions.getParkInBackstage();
    }

    public AutonomousOptions.PlaceTeamArtOnSpike getPlaceTeamArtOnSpike() {
        return autonomousOptions.getPlaceTeamArtOnSpike();
    }

    public AutonomousOptions.PlacePixelsInBackstage getPlacePixelsInBackstage() {
        return autonomousOptions.getPlacePixelsInBackStage();
    }

    public AutonomousOptions.PlacePixelsOnBackdrop getPlacePixelsOnBackdrop() {
        return autonomousOptions.getPlacePixelsOnBackdrop();
    }

    public int getDelayStartSeconds() {
        return autonomousOptions.getDelayStartSeconds();
    }

    public boolean getReadyToStart() {
        return readyToStart;
    }

    private void ShowHelp() {
        teleAlliance = telemetry.addData("X = Blue, B = Red", autonomousOptions.getAllianceColor());
        teleStartPosition = telemetry.addData("D-pad left/right, select start position", autonomousOptions.getStartPosition());
        teleParkInBackstage = telemetry.addData("D-pad up to cycle park in backstage", autonomousOptions.getParkInBackstage());
        telePlaceTeamArtOnSpike = telemetry.addData("D-pad down to cycle place team art on spike", autonomousOptions.getPlaceTeamArtOnSpike());
        telePlacePixelsInBackstage = telemetry.addData("A to cycle place pixels in backstage", autonomousOptions.getPlacePixelsInBackStage());
        telePlacePixelsOnBackdrop = telemetry.addData("Y to cycle place pixels on backdrop", autonomousOptions.getPlacePixelsOnBackdrop());
        teleDelayStartSeconds = telemetry.addData("Left & Right bumpers, Delay Start", autonomousOptions.getDelayStartSeconds());
        teleReadyToStart = telemetry.addData("Ready to start: ", getReadyToStart());
        teleSavedToFile = telemetry.addData("Saved to file:", savedToFile);
        telemetry.addLine("Back button resets all options.");
    }

    // Call this in the init_loop from your opMode. It will returns true if you press the
    // game pad Start.
    public void init_loop() {
        gamePad.readButtons();
        //Set default options (ignore what was saved to the file.)
        if (gamePad.wasJustReleased(GamepadKeys.Button.BACK)) {
            resetOptions();
        }
        //Alliance Color
        if (gamePad.wasJustReleased(GamepadKeys.Button.X)) {
            autonomousOptions.setAllianceColor(AutonomousOptions.AllianceColor.Blue);
            telemetry.speak("blue");
        }

        if (gamePad.wasJustReleased(GamepadKeys.Button.B)) {
            autonomousOptions.setAllianceColor(AutonomousOptions.AllianceColor.Red);
            telemetry.speak("red");
        }
        teleAlliance.setValue(autonomousOptions.getAllianceColor());

        //Start Position
        if (gamePad.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
            autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.Right);
            telemetry.speak("start right");
        }

        if (gamePad.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.Left);
            telemetry.speak("start left");
        }
        teleStartPosition.setValue(autonomousOptions.getStartPosition());

        //Park in backstage
        if (gamePad.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
            AutonomousOptions.ParkInBackstage parkInBackstage = autonomousOptions.getParkInBackstage().getNext();
            switch (parkInBackstage) {
                case Yes:
                    telemetry.speak("park in backstage, yes");
                    break;
                case No:
                    telemetry.speak("park in backstage,, no");
                    break;
            }
            autonomousOptions.setParkInBackstage(parkInBackstage);
            teleParkInBackstage.setValue(parkInBackstage);
        }

        //Place team art on spike.
        if (gamePad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
            AutonomousOptions.PlaceTeamArtOnSpike placeTeamArtOnSpike = autonomousOptions.getPlaceTeamArtOnSpike().getNext();
            switch (placeTeamArtOnSpike) {
                case Yes:
                    telemetry.speak("place team art on spike, yes");
                    break;
                case No:
                    telemetry.speak("place team art on spike, no");
                    break;
            }
            autonomousOptions.setPlaceTeamArtOnSpike(placeTeamArtOnSpike);
            telePlaceTeamArtOnSpike.setValue(placeTeamArtOnSpike);
        }

        //Place pixels in backstage
        if (gamePad.wasJustReleased(GamepadKeys.Button.A)) {
            AutonomousOptions.PlacePixelsInBackstage placePixelsInBackstage = autonomousOptions.getPlacePixelsInBackStage().getNext();
            switch (placePixelsInBackstage) {
                case Yes:
                    telemetry.speak("place pixels in backstage, yes");
                    break;
                case No:
                    telemetry.speak("place pixels in backstage, no");
            }
            autonomousOptions.setPlacePixelsInBackStage(placePixelsInBackstage);
            telePlacePixelsInBackstage.setValue(placePixelsInBackstage);
        }

        //Place pixels on backdrop
        if (gamePad.wasJustReleased(GamepadKeys.Button.Y)) {
            AutonomousOptions.PlacePixelsOnBackdrop placePixelsOnBackdrop = autonomousOptions.getPlacePixelsOnBackdrop().getNext();
            switch (placePixelsOnBackdrop) {
                case Yes:
                    telemetry.speak("place pixels in backstage, yes");
                    break;
                case No:
                    telemetry.speak("place pixels in backstage, no");
            }
            autonomousOptions.setPlacePixelsOnBackdrop(placePixelsOnBackdrop);
            telePlacePixelsOnBackdrop.setValue(placePixelsOnBackdrop);
        }

        // Keep range within 0-15 seconds. Wrap at either end.
        if (gamePad.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            autonomousOptions.setDelayStartSeconds(autonomousOptions.getDelayStartSeconds() - 1);
            autonomousOptions.setDelayStartSeconds((autonomousOptions.getDelayStartSeconds() < 0) ? 15 : autonomousOptions.getDelayStartSeconds());
            telemetry.speak("delay start " + autonomousOptions.getDelayStartSeconds() + " seconds");
        }
        if (gamePad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            autonomousOptions.setDelayStartSeconds(autonomousOptions.getDelayStartSeconds() + 1);
            autonomousOptions.setDelayStartSeconds((autonomousOptions.getDelayStartSeconds() > 15) ? 0 : autonomousOptions.getDelayStartSeconds());
            telemetry.speak("delay start " + autonomousOptions.getDelayStartSeconds() + " seconds");
        }
        teleDelayStartSeconds.setValue(autonomousOptions.getDelayStartSeconds());

        //Have the required options been set?
        readyToStart = !(autonomousOptions.getAllianceColor() == AutonomousOptions.AllianceColor.None || autonomousOptions.getStartPosition() == AutonomousOptions.StartPosition.None);
        teleReadyToStart.setValue(readyToStart);

        //Save the options to a file if ready to start and start button is pressed.
        if (gamePad.wasJustReleased(GamepadKeys.Button.START) && getReadyToStart()) {
            SaveOptions();
            savedToFile = true;
            teleSavedToFile.setValue(true);
        }
    }

    // Default selections if driver does not select anything.
    private void resetOptions() {
        autonomousOptions.setAllianceColor(AutonomousOptions.AllianceColor.None);
        autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.None);
        autonomousOptions.setParkInBackstage(AutonomousOptions.ParkInBackstage.No);
        autonomousOptions.setPlaceTeamArtOnSpike(AutonomousOptions.PlaceTeamArtOnSpike.No);
        autonomousOptions.setPlacePixelsInBackStage(AutonomousOptions.PlacePixelsInBackstage.No);
        autonomousOptions.setPlacePixelsOnBackdrop(AutonomousOptions.PlacePixelsOnBackdrop.No);
        autonomousOptions.setDelayStartSeconds(0);
        readyToStart = false;
        savedToFile = false;
    }

    private void SaveOptions() {
        ReadWriteAutoOptions readWriteAutoOptions = new ReadWriteAutoOptions(context);
        readWriteAutoOptions.storeObject(autonomousOptions);
    }

    public AutonomousOptions getSaveAutoOptions() {
        ReadWriteAutoOptions readWriteAutoOptions = new ReadWriteAutoOptions(context);
        AutonomousOptions temp = readWriteAutoOptions.getObject();
        telemetry.addData("Start: ", temp.getStartPosition());
        telemetry.update();
        return temp;
    }
}
