package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ArmExtension {
    public class ArmPickup implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action ArmPickup() {
        return new ArmPickup();
    }

    public class ArmBackDrop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action ArmBackDrop() {
        return new ArmBackDrop();
    }

    public class ArmRetract implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action ArmRetract() {
        return new ArmRetract();
    }

    public class ArmNone implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action ArmNone() {
        return new ArmNone();
    }
}
