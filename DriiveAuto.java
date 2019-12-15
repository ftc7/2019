package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

@Autonomous(name="Red")
public class DriiveAuto extends LinearOpMode implements TeleAuto {
    private Blinky robot = new Blinky();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
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

        vuforia.initVuforia(hardwareMap, dashboard);
        vuforia.activateVuforia();
        vuforia.setFlash(true);

        telemetry.addData("ready", "ready");
        updateTelemetry(telemetry);

        while(!isStarted()) {
            TelemetryPacket packet = new TelemetryPacket();
            if(vuforia.updateVuforia()) {
                vuforiaTelemetry(vuforia, packet);
            }
            dashboard.sendTelemetryPacket(packet);
        }
        waitForStart();

        driving.polarAuto(autospeed, Math.PI / 2, 400 * clicksPerMm, this);                 // Drive to look at the blocks

        // Open the side lift grabber
        robot.sideliftgrab.setPosition(0);

        // Give Vuforia a second to locate the block
        double looptime = getRuntime();
        int position = 0;
        boolean seenblock = false;
        while(opModeIsActive() && getRuntime() < looptime + 1) {
            TelemetryPacket packet = new TelemetryPacket();

            // Locates the block and determines which position it is in
            if(vuforia.updateVuforia()) {
                vuforiaTelemetry(vuforia, packet);
                if(vuforia.translation.get(1) > 0) {
                    position = 3;
                    packet.put("position", 3);
                }
                else if(vuforia.translation.get(1) > -205) {
                    position = 2;
                    packet.put("position", 2);
                }
                else {
                    position = 1;
                    packet.put("position", 1);
                }

                seenblock = true;
            }
            // If we can't see it, it's in the far left position
            else {
                position = 1;
                packet.put("position", 1);
            }

            dashboard.sendTelemetryPacket(packet);
        }

        // Set the side lift to RUN_TO_POSITION
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition());
        robot.sidelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        vuforia.updateVuforia();

        // Drive to block
        switch(position) {
            case 1:     // block on left
                driving.polarAuto(0.5, 0, 310 * clicksPerMm, this);       // in front of block
                grabBlock();
                driving.polarAuto(0.5, Math.PI, -360 * clicksPerMm, this);    // to standard position
                break;
            case 2:
                driving.polarAuto(0.5, 0, 50 * clicksPerMm, this);       // in front of block
                grabBlock();
                driving.polarAuto(0.5, Math.PI, -100 * clicksPerMm, this);    // to standard position
                break;
            case 3:
                driving.polarAuto(0.5, Math.PI, -30 * clicksPerMm, this);       // in front of block
                grabBlock();
                //driving.polarAuto(0.5, Math.PI * 3 / 2, 0, this);    // to standard position
                break;
        }

        // Turns to the position set with turnAbs while driving across field
        //driving.turnAbs(pi);
        //driving.polarAuto(0, 0, 0, this);
        driving.turnAbs(pi);

        // Drive across the field, drop
        driving.polarAuto(autospeed, pi, 1150 * clicksPerMm, this);
        vuforia.setFlash(false);
        robot.sideliftgrab.setPosition(0);
        //robot.leftintake.setPower(-1);
        //robot.rightintake.setPower(1);

        // Drive to the foundation
        driving.polarAuto(autospeed, 2.4, 800 * clicksPerMm, this);
        // Grab the platform
        robot.platform.setPosition(0.9);

        sleep(300);

        // Turn with the platform
        driving.polarAutoTurn(autospeed, pi/2, 1300 * clicksPerMm, this, 0.5);

        driving.polarAutoTurn(autospeed, 3*pi/2, 900 * clicksPerMm, this, -0.2);

        robot.platform.setPosition(0.4);

        driving.polarAuto(autospeed, 0, 1000 * clicksPerMm, this);
    }

    /*private void driveAnywhere(double x, double y, double speed, SkystoneNav vuforia) {
        while(opModeIsActive() && !vuforia.updateVuforia());
        double dx = x - vuforia.translation.get(0);
        double dy = y - vuforia.translation.get(1);
        double theta = Math.atan2(dx, dy);
        double r = Math.sqrt(dx * dx + dy * dy);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("dx", dx);
        packet.put("dy", dy);
        packet.put("theta", theta);
        packet.put("r", r);
        sleep(500);
        dashboard.sendTelemetryPacket(packet);
        sleep(5000);
        //driving.polarAuto(speed, theta, r, this);
        driving.polarAuto(speed, Math.PI, dx, this);
        driving.polarAuto(speed, Math.PI/2, dy, this);
    }*/

    private void grabBlock() {
        driving.polarAuto(0.4, Math.PI / 2, 310 * clicksPerMm, this);                   // to block
        robot.sideliftgrab.setPosition(0.6);
        sleep(500);
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition() + 100);
        robot.sidelift.setPower(0.5);
        sleep(500);
        driving.polarAuto(0.4, Math.PI * 3/2, 400 * clicksPerMm, this);                  // away from block
    }

    private void vuforiaTelemetry(SkystoneNav instance, TelemetryPacket packet) {
        packet.put("x", instance.translation.get(0));
        packet.put("y", instance.translation.get(1));
        packet.put("z", instance.translation.get(2));
        packet.put("roll", instance.rotation.firstAngle);
        packet.put("pitch", instance.rotation.secondAngle);
        packet.put("heading", instance.rotation.thirdAngle);
        packet.fieldOverlay()
                .setStrokeWidth(1)
                .setStroke("black")
                .fillCircle(vuforia.translation.get(0) / mmPerInch, vuforia.translation.get(1) / mmPerInch, 5)
                .setFill("black");
    }

    public void updateAuto(TelemetryPacket packet) {
        dashboard.sendTelemetryPacket(packet);
        robot.updateGyro();
        driving.gyro(Math.toRadians(robot.angles.thirdAngle));
    }
}
