package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmExtension {

    private RobotHardware robotHardware;

    public ArmExtension(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }


    public class ArmPickup implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robotHardware.ArmMotor.setTargetPosition(-480);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(0.5);

            return false;
        }
    }

    public Action ArmPickup() {
        return new ArmPickup();
    }

    public class ArmBackDrop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            robotHardware.ArmMotor.setTargetPosition(-1050);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(-0.5);

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
