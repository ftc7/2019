package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

@Autonomous
public class DriiveAuto extends LinearOpMode implements TeleAuto {
    private DriivePrototypeHardware robot = new DriivePrototypeHardware();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Driive driving = new Driive();
    private SkystoneNav vuforia = new SkystoneNav();

    public void runOpMode() {
        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double pi = Math.PI;
        double[] angles = {pi/4, pi*3/4, pi*5/4, pi*7/4};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {}

        vuforia.initVuforia(hardwareMap, dashboard);
        vuforia.activateVuforia();

        waitForStart();

        while(opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();

            if(vuforia.updateVuforia()) {
                packet.put("x", vuforia.translation.get(0) / mmPerInch);
                packet.put("y", vuforia.translation.get(1) / mmPerInch);
                packet.put("z", vuforia.translation.get(2) / mmPerInch);
                packet.put("roll", vuforia.rotation.firstAngle);
                packet.put("pitch", vuforia.rotation.secondAngle);
                packet.put("heading", vuforia.rotation.thirdAngle);

                packet.fieldOverlay()
                        .setStrokeWidth(1)
                        .setStroke("black")
                        .fillCircle(vuforia.translation.get(0) / mmPerInch, vuforia.translation.get(1) / mmPerInch, 5)
                        .setFill("black");
            }

            dashboard.sendTelemetryPacket(packet);
        }

        /*driving.polarAuto(1, 0, 2000, this);
        sleep(1000);
        driving.polarAuto(1, Math.PI/2, 2000, this);*/
    }

    public void updateAuto(TelemetryPacket packet) {
        dashboard.sendTelemetryPacket(packet);
        robot.updateGyro();
        driving.gyro(Math.toRadians(robot.angles.thirdAngle));
    }
}
