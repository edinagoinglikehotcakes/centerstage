package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class BasicAuto extends OpMode {
    private RobotHardware robotHardware;
    private MotorControl motorControl;
    //define variables
    double maxPower = 0.9;
    double denominator = 0;
    //Joystick movement
    double axial   = 0;
    double lateral =  0;
    double yaw     =  0;


    @Override
    public void init() {
        robotHardware = new RobotHardware(this);
        robotHardware.init();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        motorControl.drive( axial, lateral, yaw, maxPower, denominator);

    }

    @Override
    public void loop() {

    }
}

