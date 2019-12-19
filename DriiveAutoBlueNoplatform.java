package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Blue no platform", group="blue")
public class DriiveAutoBlueNoplatform extends LinearOpMode implements TeleAuto {
    private Blinky robot = new Blinky();
    private Driive driving = new Driive();
    private SkystoneNav vuforia = new SkystoneNav();
    private double clicksPerMm = 1.64;
    private double autospeed = 0.8;

    public void runOpMode() {

        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double pi = Math.PI;
        double[] angles = {pi / 4, pi * 3 / 4, pi * 5 / 4, pi * 7 / 4};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {
        }

        vuforia.initVuforia(hardwareMap);
        vuforia.activateVuforia();
        vuforia.setFlash(true);

        /*while(!isStarted()) {
            if(vuforia.updateVuforia()) {
            }
        }*/
        telemetry.addData("ready", "ready");
        updateTelemetry(telemetry);

        waitForStart();

        driving.polarAuto(autospeed, Math.PI / 2, 450 * clicksPerMm, this);                 // Drive to look at the blocks

        // Open the side lift grabber
        robot.sideliftgrab.setPosition(0);

        // Give Vuforia a second to locate the block
        double looptime = getRuntime();
        int position = 0;
        boolean seenblock = false;
        while(opModeIsActive() && getRuntime() < looptime + 1) {

            // Locates the block and determines which position it is in
            if(vuforia.updateVuforia()) {
                if(vuforia.translation.get(1) < 0) {
                    position = 3;
                }
                else if(vuforia.translation.get(1) < 205) {
                    position = 2;
                }
                else {
                    position = 1;
                }

                seenblock = true;
            }
            // If we can't see it, it's in the far left position
            else {
                position = 1;
            }

        }

        // Set the side lift to RUN_TO_POSITION
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition());
        robot.sidelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        vuforia.updateVuforia();

        // Drive to block
        switch(position) {
            case 1:     // block on left
                driving.polarAuto(0.5, Math.PI, 220 * clicksPerMm, this);       // in front of block
                grabBlock();
                driving.polarAuto(0.5, 0, -490 * clicksPerMm, this);    // to standard position
                break;
            case 2:
                driving.polarAuto(0.5, Math.PI, 30 * clicksPerMm, this);       // in front of block
                grabBlock();
                driving.polarAuto(0.5, 0, -120 * clicksPerMm, this);    // to standard position
                break;
            case 3:
                driving.polarAuto(0.5, 0, -60 * clicksPerMm, this);       // in front of block
                grabBlock();
                //driving.polarAuto(0.5, Math.PI * 3 / 2, 0, this);    // to standard position
                break;
        }

        // Turns to the position set with turnAbs while driving across field
        //driving.turnAbs(pi);
        //driving.polarAuto(0, 0, 0, this);

        // Drive across the field, drop
        driving.polarAuto(autospeed, 0, 1000 * clicksPerMm, this);
        vuforia.setFlash(false);
        robot.sideliftgrab.setPosition(0);
        driving.polarAuto(0.5, pi/2, 100 * clicksPerMm, this);
        driving.polarAuto(0.5, pi*3/2, 100 * clicksPerMm, this);
        //robot.leftintake.setPower(-1);
        //robot.rightintake.setPower(1);

        // Drive to the foundation
        /*driving.polarAuto(autospeed, 0.4, 600 * clicksPerMm, this);
        // Grab the platform
        robot.platform.setPosition(0.9);

        sleep(300);

        // Turn with the platform
        driving.polarAutoTurn(autospeed, pi/2, -1200 * clicksPerMm, this, -0.5);

        driving.polarAutoTurn(autospeed, 3*pi/2, -1000 * clicksPerMm, this, 0.0);

        robot.platform.setPosition(0.4);
        sleep(200);

        // Park
        driving.polarAuto(autospeed, 3 * pi / 4, 600 * clicksPerMm, this);

        robot.sideliftgrab.setPosition(0);*/

        driving.polarAuto(autospeed, pi, 500 * clicksPerMm, this);
    }

    /*private void driveAnywhere(double x, double y, double speed, SkystoneNav vuforia) {
        while(opModeIsActive() && !vuforia.updateVuforia());
        double dx = x - vuforia.translation.get(0);
        double dy = y - vuforia.translation.get(1);
        double theta = Math.atan2(dx, dy);
        double r = Math.sqrt(dx * dx + dy * dy);
        sleep(500);
        sleep(5000);
        //driving.polarAuto(speed, theta, r, this);
        driving.polarAuto(speed, Math.PI, dx, this);
        driving.polarAuto(speed, Math.PI/2, dy, this);
    }*/

    private void grabBlock() {
        driving.polarAuto(0.4, Math.PI / 2, 330 * clicksPerMm, this);                   // to block
        robot.sideliftgrab.setPosition(0.6);
        sleep(500);
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition() + 100);
        robot.sidelift.setPower(0.5);
        sleep(500);
        driving.polarAuto(0.4, Math.PI * 3/2, 200 * clicksPerMm, this);                  // away from block
    }
/*
                .setStrokeWidth(1)
                .setStroke("black")
                .fillCircle(vuforia.translation.get(0) / mmPerInch, vuforia.translation.get(1) / mmPerInch, 5)
                .setFill("black");
    }*/

    public void updateAuto() {
        robot.updateGyro();
        driving.gyro(Math.toRadians(robot.angles.thirdAngle));
        telemetry.addData("gyro", robot.angles.thirdAngle);
        updateTelemetry(telemetry);
    }
}
