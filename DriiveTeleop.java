package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
class config {
    //public static int servoPower = 0;
}

@TeleOp(name = "Driive v2")
public class DriiveTeleop extends OpMode {
    private DriivePrototypeHardware robot = new DriivePrototypeHardware();
    //private gpPrev1 prev1 = new gpPrev1();
    private Gamepad prev1 = new Gamepad();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Driive driving = new Driive();

    private double angle1 = 0.0;

    public void init() {
        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double pi = Math.PI;
        double[] angles = {pi/4, pi*3/4, pi*5/4, pi*7/4};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {
        }
        telemetry.addData("Initialized", "Initialized");
        telemetry.update();
    }

    public void init_loop() {
        driving.gyro(Math.toRadians(-robot.angles.thirdAngle));
        TelemetryPacket packet = new TelemetryPacket();
        driving.updateTelemetry(packet);
        dashboard.sendTelemetryPacket(packet);
    }

    public void loop() {
        double currentAngle = Math.toRadians(-robot.angles.thirdAngle);

        // -- DRIVING --

        robot.updateGyro();

        driving.cartesian(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        driving.gyro(currentAngle);
        driving.driive();

        // -- CONTROLS --

        if(gamepad1.left_bumper) {
            angle1 = currentAngle;
        }
        if(gamepad1.right_bumper) {
            driving.turnAbs(angle1);
        }

        if(gamepad1.x && !prev1.x) {
            driving.fieldCentric = !driving.fieldCentric;
        }
        if(gamepad1.start && !prev1.start) {
            driving.resetZero();
        }

        try {
            prev1.copy(gamepad1);
        } catch(RobotCoreException e) {}

        // -- TELEMETRY --

        TelemetryPacket packet =  new TelemetryPacket();

        packet.put("fieldCentric", driving.fieldCentric);
        packet.put("righteous", driving.righteous);
        packet.put("speed", driving.speed);

        driving.updateTelemetry(packet);

        dashboard.sendTelemetryPacket(packet);

        // -- TESTING MOTORS --

        if(gamepad1.left_bumper) {
            robot.servo.setPower(1);
            robot.servo1.setPower(-1);
        } else if(gamepad1.right_bumper) {
            robot.servo.setPower(-1);
            robot.servo1.setPower(1);
        } else {
            robot.servo.setPower(0);
            robot.servo1.setPower(0);
        }
    }
}