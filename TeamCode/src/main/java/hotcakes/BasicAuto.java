package hotcakes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

@Autonomous
public class BasicAuto extends OpMode {
    private RobotHardware robotHardware;
    private ElapsedTime runTime;
    private MotorControl motorControl;
    //define variables
    double maxPower = 0.9;
    double denominator = 0;
    //Joystick movement
    double axial = 1;
    double lateral = 0;
    double yaw = 0;


    @Override
    public void init() {
        runTime = new ElapsedTime();
        robotHardware = new RobotHardware(this);
        robotHardware.init();
        motorControl = new MotorControl(robotHardware);
        telemetry.addData("init", "");
        telemetry.update();
        robotHardware.Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.Frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.Backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.Backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        telemetry.addData("start", "");
        telemetry.update();
        runTime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("runTime", runTime.seconds());
        telemetry.update();
        motorControl.drive(axial, lateral, yaw, maxPower);
//        motorControl.drive(0.1, 0.2, 0.3, -0.9, 0.5);
    }
}

