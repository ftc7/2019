package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

@Autonomous(name="full red", group="bred")
public class DriiveAutoRed extends LinearOpMode implements TeleAuto {
    private Blinky robot = new Blinky();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Driive driving = new Driive();
    private SkystoneNav vuforia = new SkystoneNav();
    private double clicksPerMm = .6;
    private double autospeed = 1;

    public void runOpMode() {
        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double[] angles = {PI / 4, PI * 3 / 4, PI * 5 / 4, PI * 7 / 4};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {
        }

        driving.speed = 10;

        vuforia.initVuforia(hardwareMap, dashboard);
        vuforia.activateVuforia();
        vuforia.setFlash(true);

        /*while(!isStarted()) {
            TelemetryPacket packet = new TelemetryPacket();
            if(vuforia.updateVuforia()) {
                vuforiaTelemetry(vuforia, packet);
            }
            dashboard.sendTelemetryPacket(packet);
        }*/
        telemetry.addData("ready", "ready");
        updateTelemetry(telemetry);

        waitForStart();

        // Drive to look at the blocks
        driving.polarAuto(autospeed, PI / 2, 450 * clicksPerMm, this, 100);
        driving.stopWheels();

        // Open the side lift grabber
        robot.sideliftgrab.setPosition(0);

        // Give Vuforia a second to locate the block
        double looptime = getRuntime();
        int position = 0;
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
        vuforia.setFlash(false);

        // Drive to block
        switch(position) {
            case 1:     // block on left
                driving.polarAuto(0.5, 0, 310 * clicksPerMm, this);       // in front of block
                grabBlock();
                driving.polarAuto(0.5, PI, -360 * clicksPerMm, this, 0);    // to standard position
                break;
            case 2:
                driving.polarAuto(0.5, 0, 50 * clicksPerMm, this);       // in front of block
                grabBlock();
                driving.polarAuto(0.5, PI, -100 * clicksPerMm, this, 0);    // to standard position
                break;
            case 3:
                driving.polarAuto(0.5, PI, -30 * clicksPerMm, this);       // in front of block
                grabBlock();
                //driving.polarAuto(0.5, Math.PI * 3 / 2, 0, this);    // to standard position
                break;
        }

        // Turns to the position set with turnAbs while driving across field
        //driving.turnAbs(pi);
        //driving.polarAuto(0, 0, 0, this);

        // Drive across the field, drop
        driving.polarAuto(autospeed / 2, PI, 1000 * clicksPerMm, this, 0, false, false);
        robot.sidelift.setTargetPosition(robot.sidelift.getTargetPosition() - 1000);
        driving.polarAuto(autospeed / 2, PI, 1000 * clicksPerMm, this, 200, false, true);
        driving.stopWheels();
        sleep(100);
        //driving.polarAuto(0, 0, 0, this);
        //robot.leftintake.setPower(-1);
        //robot.rightintake.setPower(1);

        // Drive to the foundation
        driving.polarAuto(autospeed / 4, PI / 2, 20, this, 20, true, false);
        double runtime = getRuntime();
        while(!(robot.distance_blockplat.getDistance(CM) < 50) && runtime > getRuntime() - 1 && opModeIsActive());        // gross logic necessary for v2 sensor
        driving.stopWheels();
        sleep(500);

        // Let go of block
        robot.sideliftgrab.setPosition(0);
        sleep(200);

        // Back away from platform
        driving.polarAuto(autospeed / 3, 3 * PI / 2, 100, this, 0);
        // Lower lift
        robot.sidelift.setTargetPosition(0);
        // park
        driving.polarAuto(autospeed / 2, 0, 600, this, 100);
        driving.polarAuto(0, 0, 0, this);
        // Turn towards platform
        driving.speed = 4;
        driving.turnAbs(.95 * 3 * PI / 2);
        driving.polarAuto(0, 0, 0, this);
        driving.stopWheels();
        driving.speed = 10;
        // Drive to platform
        robot.platform.setPosition(0.5);
        driving.polarAuto(autospeed / 4, 3 * PI / 4, 20, this, 20);
        while(robot.distance_platform.getDistance(CM) > 4.5 && opModeIsActive());
        driving.stopWheels();
        // Grab the platform
        robot.platform.setPosition(0.65);

        driving.turnAbs(3 * PI / 2);

        sleep(500);

        // Reposition platform
        driving.polarAuto(autospeed, 3 * PI / 2, 20, this, 0);
        while(robot.distance_unplat.getDistance(CM) > 4.5 && opModeIsActive());

        // Park
        robot.platform.setPosition(0.2);
        sleep(500);
        driving.polarAuto(autospeed, 0, 380, this);
        driving.stopWheels();
        sleep(5000);
    }

    private void grabBlock() {
        driving.polarAuto(0.2, Math.PI / 2, 300 * clicksPerMm, this, 50);                   // to block
        double runtime = getRuntime();
        while(robot.distance_alignment.getDistance(CM) > 5 && getRuntime() < runtime + 1 && opModeIsActive());
        driving.stopWheels();
        sleep(300);
        robot.sideliftgrab.setPosition(0.6);
        sleep(500);
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition() - 200);
        robot.sidelift.setPower(0.5);
        sleep(500);
        driving.setBrake(true);
        driving.polarAuto(0.4, Math.PI * 3/2, 390 * clicksPerMm, this);                  // away from block
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
        driving.gyro(robot.angles.thirdAngle);
    }
}
