package hotcakes;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class MotorControl {
    private RobotHardware robotHardware;
    public MotorControl(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }
    public void drive (double axial, double lateral, double yaw, double maxPower,
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

