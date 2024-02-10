package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmLift {
    private boolean initialized = false;
    private RobotHardware robotHardware;

    public ArmLift(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public class LiftPickupAngle implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                robotHardware.ArmAngle.setTargetPosition(35);
                robotHardware.ArmAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotHardware.ArmAngle.setPower(0.5);
                initialized = true;
            }

            double pos = robotHardware.ArmAngle.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            if (pos < robotHardware.ArmAngle.getTargetPosition()) {
                return true;
            } else {
                robotHardware.ArmAngle.setPower(0);
                return false;
            }
        }
    }

    public Action liftPickup() {
        return new LiftPickupAngle();
    }

    public class LiftBackDropAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action LiftBackDropAngle() {
        return new LiftBackDropAngle();
    }

    public class LiftDriveAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action LiftDriveAngle() {
        return new LiftDriveAngle();
    }

    public class LiftMaxAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public Action LiftMaxAngle() {
        return new LiftMaxAngle();
    }
}

