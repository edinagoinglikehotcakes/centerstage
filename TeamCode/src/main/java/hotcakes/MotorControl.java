package hotcakes;

/*
This class manages the actions of hardware defined in RobotHardware.
Motor control should go here.
 */
public class MotorControl {
    private RobotHardware robotHardware;

    public MotorControl(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public void drive(double axial, double lateral, double yaw, double maxPower) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(axial * lateral), 1);
        robotHardware.Frontleft.setPower(((axial + lateral + yaw) / denominator) * maxPower);
        robotHardware.Backleft.setPower((((axial - lateral) + yaw) / denominator) * maxPower);
        robotHardware.Frontright.setPower((((axial - lateral) - yaw) / denominator) * maxPower);
        robotHardware.Backright.setPower((((axial + lateral) - yaw) / denominator) * maxPower);
    }
}

