package hotcakes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmLift {
    private boolean initialized = false;
    private OpMode opMode;
    private DcMotorEx ArmAngle;

    public ArmLift(OpMode opMode) {
        this.opMode = opMode;
        ArmAngle = opMode.hardwareMap.get(DcMotorEx.class, "Armangle");
        ArmAngle.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ArmAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public class LiftPickupAngle implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                ArmAngle.setTargetPosition(35);
                ArmAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmAngle.setPower(0.4);
                initialized = true;
            }

            double pos = ArmAngle.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            if (pos < ArmAngle.getTargetPosition()) {
                return true;
            } else {
                ArmAngle.setPower(0);
                return false;
            }
        }
    }

    public Action liftPickup() {return new LiftPickupAngle();}

    public class LiftBackDropAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                ArmAngle.setTargetPosition(735);
                ArmAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmAngle.setPower(0.5);
            }
            double pos = ArmAngle.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            if (pos < ArmAngle.getTargetPosition()) {
                return true;
            } else {
                ArmAngle.setPower(0);
//TODO                Might have to add break mode
                return false;
            }
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

