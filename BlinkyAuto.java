package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.PI;

public abstract class BlinkyAuto extends LinearOpMode implements TeleAuto {
    private Blinky robot = new Blinky();
    private Driive driving = new Driive();
    private SkystoneNav vuforia = new SkystoneNav();
    private double clicksPerMm = .6;
    private double autospeed = .5;
    private FailsafeDashboard dashboard = new FailsafeDashboard();

    private void initRobot() {
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
     * Drive forward, grab the block, pick it up, drive backwards a set amount
     *
     * @param back was 280
     */
    private void grabBlock(double back) {
        // to block
        driving.polarAuto(0.3, Math.PI / 2, 100, this, 0, false, false);
        while(opModeIsActive() && robot.distance_alignment.getDistance(DistanceUnit.CM) > 4 && !(robot.distance_blockplat.getDistance(DistanceUnit.CM) < 30));
        driving.stopWheels();

        //grab block
        robot.sideliftgrab.setPosition(0.8);
        sleep(500);
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition() - 100);
        robot.sidelift.setPower(0.5);
        sleep(500);

        // away from block
        driving.polarAuto(0.3, Math.PI * 3/2, back * clicksPerMm, this, 100);
    }

    /**
     * Detects position of Skystone
     *
     * @param timeout Time, in seconds, to pause for recognition
     * @return The position of the block, from 1 (audience) to 3 (foundation)
     */
    private int vuforiaDetection(double timeout) {
        // Give Vuforia a second to locate the block
        double looptime = getRuntime();
        int position = 0;
        while(opModeIsActive() && getRuntime() < looptime + timeout) {
            // Locates the block and determines which position it is in
            if(vuforia.updateVuforia()) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("x", vuforia.translation.get(1));
                dashboard.sendTelemetryPacket(packet);
                if(vuforia.translation.get(1) > 0) position = 3;
                else if(vuforia.translation.get(1) > -205) position = 2;
                else position = 1;
            }
            // If we can't see it, it's in the far left position
            else position = 1;
        }

        return position;
    }

    public void updateAuto(TelemetryPacket packet) {
        dashboard.sendTelemetryPacket(packet);
        robot.updateGyro();
        driving.gyro(robot.angles.thirdAngle);
    }
}
