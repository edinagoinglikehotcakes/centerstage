package hotcakes;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class MotorControl {
    //    set limits
    private final int ARM_TURN_LIMIT = 200;
    private final int ARM_LIMIT = 700;
    private final double TURN_SPEED = 0.3;
    private final double ARM_SPEED = 0.6;

    private RobotHardware robotHardware;

    //    Which direction the arm is currently going
    public enum armDirection {
        LEFT,
        RIGHT,
        STOP,
    }

    public enum armMovingDirection {
        UP,
        DOWN,
        NONE,
    }

    public MotorControl(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public void rotateArm(armDirection direction) {
//        Set direction of arm and call Enum and sets limits to the arm rotation
        if (direction == armDirection.LEFT) {
            if (robotHardware.TurnMotor.getCurrentPosition() >= -ARM_TURN_LIMIT) {
                robotHardware.TurnMotor.setPower(-TURN_SPEED);
            } else {
                robotHardware.TurnMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                robotHardware.TurnMotor.setPower(0);
            }
        }

        if (direction == armDirection.RIGHT) {
            if (robotHardware.TurnMotor.getCurrentPosition() <= ARM_TURN_LIMIT) {
                robotHardware.TurnMotor.setPower(TURN_SPEED);
            } else {
                robotHardware.TurnMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                robotHardware.TurnMotor.setPower(0);
            }
        }
        if (direction == armDirection.STOP) {
            robotHardware.TurnMotor.setPower(0);
            robotHardware.TurnMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

    //    ARM MOVEMENT FOR UP AND DOWN
    public void mobilizeArm(armMovingDirection movingDirection) {
        if (movingDirection == armMovingDirection.UP) {
//                        TODO Add more parameters (like IF statements for arm limits) if needed on this line
            if (robotHardware.ArmMotor.getCurrentPosition() < ARM_LIMIT) {
                robotHardware.ArmMotor.setPower(ARM_SPEED);
            } else {
                robotHardware.ArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                robotHardware.ArmMotor.setPower(0);
            }
        }
        if (movingDirection == armMovingDirection.DOWN) {
//            TODO Add more parameters (like IF statements for arm limits) if needed on this line
            if (robotHardware.ArmMotor.getCurrentPosition() > 0) {
                robotHardware.ArmMotor.setPower(-ARM_SPEED);
            } else {
                robotHardware.ArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                robotHardware.ArmMotor.setPower(0);
            }
        }
        if (movingDirection == armMovingDirection.NONE) {
            robotHardware.ArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robotHardware.ArmMotor.setPower(0);
        }
    }

    public void drive(double axial, double lateral, double yaw, double maxPower,
                      double denominator) {

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
//            If the code below does not work, comment it out. Uncomment out the code below this code below.
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(axial), Math.abs(lateral), Math.abs(yaw))), 1));
        robotHardware.Frontleft.setPower(((axial + lateral + yaw) / denominator) * maxPower);
        robotHardware.Backleft.setPower((((axial - lateral) + yaw) / denominator) * maxPower);
        robotHardware.Frontright.setPower((((axial - lateral) - yaw) / denominator) * maxPower);
        robotHardware.Backright.setPower((((axial + lateral) - yaw) / denominator) * maxPower);
    }
}

