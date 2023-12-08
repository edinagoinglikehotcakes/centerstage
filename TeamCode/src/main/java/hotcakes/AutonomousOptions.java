package hotcakes;

import java.io.Serializable;

/**
 * This class stores autonomous options for use by autonomous or teleop op modes.
 * It is currently set for the Centerstage season but can be modified for upcoming seasons.
 * It is a Java Serializable class so it can be saved to a file.
 */
public class AutonomousOptions implements Serializable {
    private static final long serialVersionUID = 7829136421241571165L;
    private int delayStartSeconds;
    private  AllianceColor allianceColor;
    private StartPosition startPosition;
    private ParkInBackstage parkInBackstage;
    private PlaceTeamArtOnSpike placeTeamArtOnSpike;
    private PlacePixelsInBackstage placePixelsInBackStage;
    private PlacePixelsOnBackdrop placePixelsOnBackdrop;

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public void setAllianceColor(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public StartPosition getStartPosition() {
        return startPosition;
    }

    public void setStartPosition(StartPosition startPosition) {
        this.startPosition = startPosition;
    }

    public ParkInBackstage getParkInBackstage() {
        return parkInBackstage;
    }

    public void setParkInBackstage(ParkInBackstage parkInBackstage) {
        this.parkInBackstage = parkInBackstage;
    }

    public PlaceTeamArtOnSpike getPlaceTeamArtOnSpike() {
        return placeTeamArtOnSpike;
    }

    public void setPlaceTeamArtOnSpike(PlaceTeamArtOnSpike placeTeamArtOnSpike) {
        this.placeTeamArtOnSpike = placeTeamArtOnSpike;
    }

    public PlacePixelsInBackstage getPlacePixelsInBackStage() {
        return placePixelsInBackStage;
    }

    public void setPlacePixelsInBackStage(PlacePixelsInBackstage placePixelsInBackStage) {
        this.placePixelsInBackStage = placePixelsInBackStage;
    }

    public PlacePixelsOnBackdrop getPlacePixelsOnBackdrop() {
        return placePixelsOnBackdrop;
    }

    public void setPlacePixelsOnBackdrop(PlacePixelsOnBackdrop placePixelsOnBackdrop) {
        this.placePixelsOnBackdrop = placePixelsOnBackdrop;
    }

    public int getDelayStartSeconds() {
        return delayStartSeconds;
    }

    public void setDelayStartSeconds(int delayStartSeconds) {
        this.delayStartSeconds = delayStartSeconds;
    }

    public String toString() {
        return "AllianceColor: " + getAllianceColor().toString() + "\nStartLocation: " + getStartPosition().toString();
    }

    /*
     * Alliance color. Default to None so driver must select it.
     */
    public enum AllianceColor {
        None,
        Red,
        Blue
    }

    /*
     * Where do we start the robot
     * Right is on the right of your substation.
     * Left is on the left of your substation.
     */
    public enum StartPosition {
        None,
        Left,
        Right
    }

    /*
     * Yes means park on the signal zone.
     * Default is No.
     */
    public enum ParkInBackstage {
        No,
        Yes;

        public ParkInBackstage getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
     * Yes means place cone in the terminal.
     *  No is the default.
     */
    public enum PlaceTeamArtOnSpike {
        No,
        Yes;

        public PlaceTeamArtOnSpike getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    public enum PlacePixelsInBackstage {
        No,
        Yes;

        public PlacePixelsInBackstage getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    public enum PlacePixelsOnBackdrop {
        No,
        Yes;

        public PlacePixelsOnBackdrop getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }
}
