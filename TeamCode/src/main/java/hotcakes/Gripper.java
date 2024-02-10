package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private OpMode opMode;
    private Servo GripperLeft;
    private Servo GripperRight;

    public Gripper(OpMode opmode) {
        this.opMode=opmode;
        GripperLeft = opMode.hardwareMap.get(Servo.class, "gripperleft");
        GripperRight = opMode.hardwareMap.get(Servo.class, "gripperright");
    }

    public class GripperLeftOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            GripperLeft.setPosition(0.78);
            return false;
        }
    }

    public Action GripperLeftOpen() {
        return new GripperLeftOpen();
    }

    public class GripperLeftClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            GripperLeft.setPosition(0.66);

            return false;
        }
    }

    public Action GripperLeftClose() {
        return new GripperLeftClose();
    }

    public class GripperRightOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            GripperRight.setPosition(0.07);

            return false;
        }
    }

    public Action GripperRightOpen() {
        return new GripperRightOpen();
    }

    public class GripperRightClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            GripperRight.setPosition(0.20);

            return false;
        }
    }

    public Action GripperRightClose() {
        return new GripperRightClose();
    }
}
