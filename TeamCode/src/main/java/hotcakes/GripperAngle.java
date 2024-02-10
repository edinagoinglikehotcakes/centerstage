package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class GripperAngle {
    private OpMode opMode;
    private Servo GripperAngle;

    public GripperAngle(OpMode opMode) {

        this.opMode = opMode;
        GripperAngle = opMode.hardwareMap.get(Servo.class, "gripperangle");
    }

    public class GripperPickup implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            GripperAngle.setPosition(0.3);
            return false;
        }
    }

    public Action GripperPickup() {
        return new GripperPickup();
    }
    public class GripperRetract implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            GripperAngle.setPosition(0.13);
            return false;
        }
    }

    public Action GripperRetract() {
        return new GripperRetract();
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
