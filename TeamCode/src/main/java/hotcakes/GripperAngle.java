package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class GripperAngle {
    private RobotHardware robotHardware;

    public GripperAngle(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public class GripperPickup implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action GripperPickup() {
        return new GripperPickup();
    }

    public class GripperBackDrop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action GripperBackDrop() {
        return new GripperBackDrop();
    }
}
