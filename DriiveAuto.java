package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

@Autonomous
public class DriiveAuto extends LinearOpMode implements TeleAuto {
    private Blinky robot = new Blinky();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Driive driving = new Driive();
    private SkystoneNav vuforia = new SkystoneNav();
    private double clicksPerMm = 1.64;

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

        waitForStart();

        driving.polarAuto(0.7, Math.PI / 2, 365 * clicksPerMm, this);                 // Drive to look at the blocks

        // Open the side lift grabber
        robot.sideliftgrab.setPosition(0);

        // Give Vuforia a second to locate the block
        double looptime = getRuntime();
        int position = 0;
        boolean seenblock = false;
        while(opModeIsActive() && getRuntime() < looptime + 20) {
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

        sleep(10000);

        // Set the side lift to RUN_TO_POSITION
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition());
        robot.sidelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        vuforia.updateVuforia();

        // Drive to block
        if(seenblock) {
            driving.polarAuto(0.3, 0, -vuforia.translation.get(2) * clicksPerMm, this);
        }
        else {
            driving.polarAuto(0.3, 0, 307 * clicksPerMm, this);
        }
        grabBlock();
        // Drive to in front of the end block
        driving.polarAuto(0.3, pi, ((3 - position) * 205) * clicksPerMm, this);

        // Turns to the position set with turnAbs while driving across field
        driving.turnAbs(pi);
        driving.polarAuto(0, 0, 0, this);
        driving.polarAuto(1, pi, 1400, this);
        robot.sideliftgrab.setPosition(0.4);

        // Find our current position using Vuforia
        do {
            TelemetryPacket packet = new TelemetryPacket();
            vuforiaTelemetry(vuforia, packet);
            dashboard.sendTelemetryPacket(packet);
        } while(opModeIsActive() && !vuforia.updateVuforia());

        sleep(10000);

        driveAnywhere(1220, -900, 0.5, vuforia);

        robot.platform.setPosition(0.9);
    }

    private void driveAnywhere(double x, double y, double speed, SkystoneNav vuforia) {
        double dx = x - vuforia.translation.get(2);
        double dy = y - vuforia.translation.get(1);
        double theta = Math.atan2(dx, dy);
        double r = Math.sqrt(dx * dx + dy * dy);
        driving.polarAuto(speed, theta, r, this);
    }

    private void grabBlock() {
        driving.polarAuto(0.4, Math.PI / 2, 330 * clicksPerMm, this);                   // to block
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
