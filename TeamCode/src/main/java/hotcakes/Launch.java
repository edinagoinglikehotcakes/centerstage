package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class Launch {
    private RobotHardware robotHardware;

    public Launch(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public class LaunchDrone implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action LaunchDrone() {
        return new LaunchDrone();
    }

    public class LaunchRest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action LaunchRest() {
        return new LaunchRest();
    }

    public class LaunchAngleWaiting implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action LaunchAngleRest() {
        return new LaunchAngleWaiting();
    }

    public class LaunchAngle implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
    public Action LaunchAngle(){
        return new LaunchAngle();
    }
}
