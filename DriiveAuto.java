package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

@Autonomous
public class DriiveAuto extends LinearOpMode implements TeleAuto {
    private Blinky robot = new Blinky();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Driive driving = new Driive();
    private SkystoneNav vuforia = new SkystoneNav();

    public void runOpMode() {
        double clicksPerMm = 1.64;

        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double pi = Math.PI;
        double[] angles = {pi / 4, pi * 3 / 4, pi * 5 / 4, pi * 7 / 4};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {
        }

        vuforia.initVuforia(hardwareMap, dashboard);
        vuforia.activateVuforia(true);

        waitForStart();

        driving.polarAuto(0.7, Math.PI / 2, 600, this);                 // Drive to look at the blocks

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

        /*switch(position) {
        case 1:     // block on left
            driving.polarAuto(0.5, 0, 375, this);                   // in front of block
            grabBlock();
            driving.polarAuto(1, Math.PI, 425, this);               // to standard position
            break;
        case 2:
            driving.polarAuto(0.5, 0, 100, this);                   // in front of block
            grabBlock();
            driving.polarAuto(1, Math.PI, 150, this);               // to standard position
            break;
        case 3:
            driving.polarAuto(0.5, Math.PI, 50, this);              // in front of block
            grabBlock();
            driving.polarAuto(1, Math.PI, 0, this);                 // to standard position
            break;
        }*/

        vuforia.updateVuforia();
        // Drive to block
        if(seenblock) {
            driving.polarAuto(0.3, 0, -vuforia.translation.get(1) * clicksPerMm, this);
        }
        else {
            driving.polarAuto(0.3, 0, 307 * clicksPerMm, this);
        }
        grabBlock();
        // Drive to in front of the end block
        driving.polarAuto(0.3, 0, ((3 - position) * 205) * clicksPerMm, this);

        // Turns to the position set with turnAbs while driving across field
        driving.turnAbs(Math.PI);
        driving.polarAuto(1, 3.1415, 1400, this);

        // Find our current position using Vuforia
        looptime = getRuntime();
        while(opModeIsActive() && getRuntime() < looptime + 1) {
            TelemetryPacket packet = new TelemetryPacket();
            vuforiaTelemetry(vuforia, packet);
        }

        double dx = 1200 - vuforia.translation.get(1);
        double dy = -1000 - vuforia.translation.get(2);
        double theta = Math.atan2(dx, dy);
        double r = Math.sqrt(dx * dx + dy * dy);
        driving.polarAuto(0.5, theta, r, this);

        robot.platform.setPosition(0.9);


    }

    private void grabBlock() {
        driving.polarAuto(0.4, Math.PI / 2, 500, this);                   // to block
        robot.sideliftgrab.setPosition(0.6);
        sleep(500);
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition() + 100);
        robot.sidelift.setPower(0.5);
        sleep(500);
        driving.polarAuto(0.4, Math.PI * 3/2, 500, this);                  // away from block
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
