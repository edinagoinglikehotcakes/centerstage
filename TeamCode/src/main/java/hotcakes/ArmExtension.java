package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmExtension {

    private boolean initialized = false;
    private OpMode opMode;
    private DcMotorEx ArmMotor;

    public ArmExtension(OpMode opMode) {
        this.opMode=opMode;
        ArmMotor = opMode.hardwareMap.get(DcMotorEx.class, "Armmotor");

    }


    public class ArmPickup implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                ArmMotor.setTargetPosition(-480);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(0.5);
                initialized = true;
            }
            double pos = ArmMotor.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            if (pos > ArmMotor.getTargetPosition()) {
                return true;
            } else {
                ArmMotor.setPower(0);
                return false;
            }

        }
    }

    public Action ArmPickup() {
        return new ArmPickup();
    }

    public class ArmBackDrop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            ArmMotor.setTargetPosition(-1050);
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmMotor.setPower(-0.5);

            return false;
        }
    }

    public Action ArmBackDrop() {
        return new ArmBackDrop();
    }

    public class ArmRetract implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ArmMotor.setTargetPosition(-40);
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmMotor.setPower(-0.5);
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
