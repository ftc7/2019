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

        driving.polarAuto(0.7, Math.PI / 2, 500, this);

        double looptime = getRuntime();

        int position;

        while(opModeIsActive() && getRuntime() < looptime + 2) {
            TelemetryPacket packet = new TelemetryPacket();

            // Locates the block and determines which position it is in
            if(vuforia.updateVuforia()) {
                vuforiaTelemetry(vuforia, packet);
                if(vuforia.translation.get(1) > 0) {
                    position = 3;
                    packet.put("position", 3);
                }
                else if(vuforia.translation.get(1) / mmPerInch > -10) {
                    position = 2;
                    packet.put("position", 2);
                }
                else {
                    position = 1;
                    packet.put("position", 1);
                }
            } else {
                position = 1;
                packet.put("position", 1);
            }

            dashboard.sendTelemetryPacket(packet);
        }

        robot.sideliftgrab.setPosition(0);
        robot.sidelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        switch(position) {
        case 1:     // block on left
            driving.polarAuto(0.5, Math.PI * 3/2, 200, this);       // in front of block
            grabBlock();
            driving.polarAuto(0.5, Math.PI * 3 / 2, -200, this);    // to standard position
            break;
        case 2:
            driving.polarAuto(0.5, Math.PI * 3/2, 50, this);       // in front of block
            grabBlock();
            driving.polarAuto(0.5, Math.PI * 3 / 2, -50, this);    // to standard position
            break;
        case 3:
            driving.polarAuto(0.5, Math.PI * 3/2, -50, this);       // in front of block
            grabBlock();
            driving.polarAuto(0.5, Math.PI * 3 / 2, 0, this);    // to standard position
            break;
        }

        driving.turnAbs(Math.PI);
        driving.polarAuto(1, Math.PI, 2000, this);   // Turns to the position set with turnAbs
    }

    void grabBlock() {
        driving.polarAuto(0.4, 0, 200, this);                   // to block
        robot.sideliftgrab.setPosition(0.6);
        sleep(500);
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition() + 80);
        robot.sidelift.setPower(0.5);
        sleep(500);
        driving.polarAuto(0.4, 0, -200, this);                  // away from block
    }

    void vuforiaTelemetry(SkystoneNav instance, TelemetryPacket packet) {
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
