package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Platform Red", group="red")
public class PlatformRed extends LinearOpMode implements TeleAuto {
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
        
        driving.polarAuto(0.7, 3*pi/2, 1000 * 1.6, this);
        robot.platform.setPosition(0.9);
        sleep(500);
        driving.polarAuto(0.7, pi/2, 900 * 1.6, this);
        robot.platform.setPosition(400);
        driving.polarAuto(0.7, pi/2, 100 * 1.6, this);
        
        // SIDE SPECIFIC
    }
    
    public void updateAuto() {}
}
