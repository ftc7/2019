package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class Park extends LinearOpMode implements TeleAuto {
    private Blinky robot = new Blinky();
    private Driive driving = new Driive();

    public void runOpMode() {

        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double pi = Math.PI;
        double[] angles = {pi / 4, pi * 3 / 4, pi * 5 / 4, pi * 7 / 4};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {
        }
        
        waitForStart();
        
        driving.polarAuto(0.5, 0, 500 * 1.6, this);
    }
    
    public void updateAuto() {}
}