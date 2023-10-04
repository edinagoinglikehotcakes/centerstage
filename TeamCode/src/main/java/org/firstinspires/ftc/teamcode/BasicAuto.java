package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class BasicAuto extends OpMode {
    private RobotHardware robotHardware;


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

    }

    @Override
    public void loop() {

    }
}
