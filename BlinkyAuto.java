package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

public abstract class BlinkyAuto extends LinearOpMode implements TeleAuto {
    Blinky robot = new Blinky();
    Driive driving = new Driive();
    SkystoneNav vuforia = new SkystoneNav();
    double clicksPerMm = .6;
    double autospeed = .7;
    FailsafeDashboard dashboard = new FailsafeDashboard();
    final boolean RED = true;
    final boolean BLUE = false;
    private boolean color;

    /**
     * Initializes vuforia, Driive, etc.
     */
    void initRobot(boolean color) {
        this.color = color;
        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double pi = Math.PI;
        double[] angles = {pi / 4, pi * 3 / 4, pi * 5 / 4, pi * 7 / 4};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {
        }

        driving.turnSpeed = 0.6;

        vuforia.initVuforia(hardwareMap, dashboard);
        vuforia.activateVuforia();
        vuforia.setFlash(true);

        telemetry.addData("ready", "ready");
        updateTelemetry(telemetry);
    }

    /**
     * Detects position of Skystone
     *
     * @param timeout Time, in seconds, to pause for recognition
     * @return The position of the block, from 1 (audience) to 3 (foundation)
     */
    int vuforiaDetection(double timeout) {
        // Give Vuforia a second to locate the block
        double looptime = getRuntime();
        int position = 0;
        while(opModeIsActive() && getRuntime() < looptime + timeout) {
            // Locates the block and determines which position it is in
            if(vuforia.updateVuforia()) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("x", vuforia.translation.get(1));
                dashboard.sendTelemetryPacket(packet);
                if(color == RED) {
                    if(vuforia.translation.get(1) > 0) position = 3;
                    else if(vuforia.translation.get(1) > -205) position = 2;
                    else position = 1;
                } else {
                    if(vuforia.translation.get(1) < 0) position = 3;
                    else if(vuforia.translation.get(1) < 205) position = 2;
                    else position = 1;
                }
            }
            // If we can't see it, it's in the far left position
            else position = 1;
        }

        vuforia.setFlash(false);
        return position;
    }

    /**
     * Drive to and pick up block from
     *
     * @param position
     */
    void toBlock(int position) {
        if(color == RED) {
            switch(position) {
                case 1:     // block on audience
                    driving.polarAuto(0.5, 0, 200 * clicksPerMm, this);       // in front of block
                    grabBlock(200);
                    driving.polarAuto(0.5, PI, 150 * clicksPerMm, this, 0, false, false);    // to standard position
                    break;
                case 2:
                    driving.polarAuto(0.1, 0, 90 * clicksPerMm, this);    // to standard position
                    grabBlock(200);
                    driving.polarAuto(0.5, PI, 140 * clicksPerMm, this, 0, false, false);    // to standard position
                    break;
                case 3:
                    driving.polarAuto(0.5, PI, 30 * clicksPerMm, this);       // in front of block
                    grabBlock(200);
                    driving.polarAuto(0.5, PI, 20 * clicksPerMm, this, 0, false, false);    // to standard position
                    break;
            }
        }
        else {
            switch(position) {
                case 1:     // block on audience
                    driving.polarAuto(0.5, PI, -160 * clicksPerMm, this);       // in front of block
                    grabBlock(150);
                    driving.polarAuto(0.5, 0, 200 * clicksPerMm, this, 0, true, false);    // to standard position
                    break;
                case 2:
                    driving.polarAuto(0.5, Math.PI, -0 * clicksPerMm, this);       // in front of block
                    grabBlock(150);
                    driving.polarAuto(0.5, 0, 40 * clicksPerMm, this, 0, true, false);    // to standard position
                    break;
                case 3:
                    driving.polarAuto(0.5, 0, 10 * clicksPerMm, this);       // in front of block
                    grabBlock(150);
                    //driving.polarAuto(0.5, Math.PI * 3 / 2, 0, this);    // to standard position
                    break;
            }
        }
    }

    /**
     * Drive forward, grab the block, pick it up, drive backwards a set amount
     *
     * @param mmBack was 280/350
     */
    void grabBlock(double mmBack) {
        // to block
        driving.polarAuto(0.3, Math.PI / 2, 100, this, 0, false, false);
        while(opModeIsActive() && robot.distance_alignment.getDistance(CM) > 4 && !(robot.distance_blockplat.getDistance(CM) < 30));
        driving.stopWheels();

        //grab block
        robot.sideliftgrab.setPosition(0.9);
        sleep(600);
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition() - 250);
        robot.sidelift.setPower(0.7);
        sleep(500);

        // away from block
        driving.polarAuto(0.3, Math.PI * 3/2, mmBack * clicksPerMm, this, 100);
    }

    /**
     * Drives to the platform using a sensor, with a timeout as a backup in case the sensor fails
     */
    void toPlatform() {
        driving.polarAuto(0.2, PI / 2, 500 * clicksPerMm, this, 0, true, false);
        double looptime = getRuntime();
        while(robot.distance_blockplat.getDistance(CM) > 100 && opModeIsActive() && getRuntime() < looptime + .5) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("blockplat", robot.distance_blockplat.getDistance(CM));
            dashboard.sendTelemetryPacket(packet);
        }
        driving.stopWheels();
    }

    public void updateAuto(TelemetryPacket packet) {
        dashboard.sendTelemetryPacket(packet);
        robot.updateGyro();
        driving.gyro(robot.angles.thirdAngle);
    }
}
