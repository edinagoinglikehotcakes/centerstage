package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class Gripper {
    private RobotHardware robotHardware;

    public Gripper(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public class GripperLeftOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robotHardware.GripperLeft.setPosition(0.78);
            return false;
        }
    }

    public Action GripperLeftOpen() {
        return new GripperLeftOpen();
    }

    public class GripperLeftClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robotHardware.GripperLeft.setPosition(0.66);

            return false;
        }
    }

    public Action GripperLeftClose() {
        return new GripperLeftClose();
    }

    public class GripperRightOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robotHardware.GripperRight.setPosition(0.07);

            return false;
        }
    }

    public Action GripperRightOpen() {
        return new GripperRightOpen();
    }

    public class GripperRightClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robotHardware.GripperRight.setPosition(0.20);

            return false;
        }
    }

    public Action GripperRightClose() {
        return new GripperRightClose();
    }
}
