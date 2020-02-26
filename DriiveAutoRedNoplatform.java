package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.PI;

@Autonomous(name="no platform red", group="bred")
public class DriiveAutoRedNoplatform extends LinearOpMode implements TeleAuto {
    private Blinky robot = new Blinky();
    private Driive driving = new Driive();
    private SkystoneNav vuforia = new SkystoneNav();
    private double clicksPerMm = .6;
    private double autospeed = .5;
    private FailsafeDashboard dashboard = new FailsafeDashboard();

    public void runOpMode() {

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

        /*while(!isStarted()) {
            if(vuforia.updateVuforia()) {
            }
        }*/
        waitForStart();
        driving.resetZero();

        // Drive to look at the blocks
        driving.polarAuto(autospeed, Math.PI / 2, 500 * clicksPerMm, this, 150, false, true);
        driving.stopWheels();

        // Open the side lift grabber
        robot.sideliftgrab.setPosition(0);

        // Give Vuforia a second to locate the block
        double looptime = getRuntime();
        int position = 0;
        while(opModeIsActive() && getRuntime() < looptime + 1) {
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

        // Set the side lift to RUN_TO_POSITION
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition());
        robot.sidelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        vuforia.updateVuforia();

        // Drive to block
        switch(position) {
            case 1:     // block on left
                driving.polarAuto(0.5, 0, 200 * clicksPerMm, this);       // in front of block
                grabBlock();
                driving.polarAuto(0.5, PI, 150 * clicksPerMm, this, 0, false, false);    // to standard position
                break;
            case 2:
                driving.polarAuto(0.1, 0, 90 * clicksPerMm, this);    // to standard position
                grabBlock();
                driving.polarAuto(0.5, PI, 140 * clicksPerMm, this, 0, false, false);    // to standard position
                break;
            case 3:
                driving.polarAuto(0.5, PI, 30 * clicksPerMm, this);       // in front of block
                grabBlock();
                driving.polarAuto(0.5, PI, 20 * clicksPerMm, this, 0, false, false);    // to standard position
                break;
        }

        // Drive to platform
        driving.polarAuto(autospeed, PI, 1000 * clicksPerMm, this, 0, true, false);
        robot.sidelift.setTargetPosition(robot.sidelift.getTargetPosition() - 1000);
        driving.polarAuto(autospeed, PI, 1400 * clicksPerMm, this, 100, false, true);

        // to platform w/ sensor
        driving.polarAuto(0.2, PI / 2, 500 * clicksPerMm, this, 0, true, false);
        looptime = getRuntime();
        while(robot.distance_platform.getDistance(DistanceUnit.CM) > 100 && opModeIsActive() && getRuntime() < looptime + 1);
        driving.stopWheels();

        // let go of block
        robot.sideliftgrab.setPosition(0);
        sleep(200);

        // away from platform
        driving.polarAuto(autospeed, 3 * PI / 2, 200 * clicksPerMm, this);

        // back across field
        robot.sidelift.setTargetPosition(0);
        driving.turnAbs(PI / 2);
        //driving.polarAuto(0, 0, 0, this);
        driving.polarAuto(autospeed, 0, 2000 * clicksPerMm, this, 100);

        // to block
        driving.polarAuto(autospeed, PI / 2, 500 * clicksPerMm, this, 100);
    }

    private void grabBlock() {
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
        driving.polarAuto(0.3, Math.PI * 3/2, 280 * clicksPerMm, this, 100);
    }

    public void updateAuto(TelemetryPacket packet) {
        dashboard.sendTelemetryPacket(packet);
        robot.updateGyro();
        driving.gyro(robot.angles.thirdAngle);
    }
}
