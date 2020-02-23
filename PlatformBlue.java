package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

@Autonomous(name="platform blue", group="platform")
public class PlatformBlue extends LinearOpMode implements TeleAuto {
    private Blinky robot = new Blinky();
    private Driive driving = new Driive();
    private FailsafeDashboard dashboard = new FailsafeDashboard();
    private double clicksPerMm = .6;
    private double autospeed = 0.4;

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

        driving.polarAuto(autospeed, pi - 0.3, 1000 * clicksPerMm, this, 100);
        //while(opModeIsActive() && !(robot.distance_blockplat.getDistance(CM) < 50));
        driving.stopWheels();
        //sleep(500);
        robot.platform.setPosition(0.65);
        driving.polarAuto(0,0,0, this);
        sleep(1500);
        driving.polarAuto(autospeed, 0, 1000 * clicksPerMm, this, 0, false, false);
        while(robot.distance_unplat.getDistance(DistanceUnit.CM) > 4 && opModeIsActive());
        robot.platform.setPosition(0.2);
        driving.polarAuto(autospeed, 0, 100 * clicksPerMm, this);
        driving.stopWheels();
        sleep(1000);

        // SIDE SPECIFIC
    }

    public void updateAuto(TelemetryPacket packet) {
        dashboard.sendTelemetryPacket(packet);
        robot.updateGyro();
        driving.gyro(robot.angles.thirdAngle);
        opModeIsActive();
    }
}
