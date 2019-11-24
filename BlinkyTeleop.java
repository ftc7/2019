package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "Blinky")
public class BlinkyTeleop extends OpMode {
    private Blinky robot = new Blinky();
    //private gpPrev1 prev1 = new gpPrev1();
    private Gamepad prev1 = new Gamepad();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Driive driving = new Driive();

    private double angle1 = 0.0;
    private boolean frontside = false;

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
        driving.gyro(Math.toRadians(robot.angles.thirdAngle));
        TelemetryPacket packet = new TelemetryPacket();
        driving.updateTelemetry(packet);
        dashboard.sendTelemetryPacket(packet);
    }

    public void loop() {
        double currentAngle = Math.toRadians(robot.angles.thirdAngle);

        // -- DRIVING --

        robot.updateGyro();

        driving.cartesian(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        driving.gyro(currentAngle);
        driving.driive();

        // -- CONTROLS --

        if(gamepad1.left_bumper) angle1 = currentAngle;
        if(gamepad1.right_bumper) driving.turnAbs(angle1);

        if(gamepad1.x && !prev1.x) driving.fieldCentric = !driving.fieldCentric;
        if(gamepad1.y && !prev1.y) driving.righteous = !driving.righteous;
        if(gamepad1.start && !prev1.start) driving.resetZero();

        if(gamepad1.a && !prev1.a) driving.speed += 2;
        if(gamepad1.b && !prev1.b) driving.speed -= 2;
        if(driving.speed > 10) driving.speed = 10;
        if(driving.speed < 2) driving.speed = 2;

        try {
            prev1.copy(gamepad1);
        } catch(RobotCoreException e) {}

        // -- SPECIAL MOTORS --

        // Bumpers run intake
        if(gamepad2.left_bumper) {
            robot.leftintake.setPower(1);
            robot.rightintake.setPower(-1);
        } else if(gamepad2.right_bumper) {
            robot.leftintake.setPower(-1);
            robot.rightintake.setPower(1);
        } else {
            robot.leftintake.setPower(0);
            robot.rightintake.setPower(0);
        }

        // Triggers run track
        robot.track.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

        // x sets frontside to front lift, y to side
        if(gamepad2.x) frontside = false;
        if(gamepad2.y) frontside = true;

        // Lift up/down based on frontside
        if(frontside) robot.frontlift.setPower(gamepad2.left_stick_y / 3);
        else robot.sidelift.setPower(gamepad2.left_stick_y / 3);

        // Right stick X controls current lift grabber
        if(frontside) robot.frontliftgrab.setPower(gamepad2.right_stick_x);
        else if(gamepad2.right_stick_x < 0) robot.sideliftgrab.setPosition(0);
        else if(gamepad2.right_stick_x > 0) robot.sideliftgrab.setPosition(1);

        // -- TELEMETRY --

        telemetry.addData("DRIVING", "");
        telemetry.addData("Field centric", driving.fieldCentric);
        telemetry.addData("Righteous", driving.righteous);
        telemetry.addData("Speed", driving.speed);
        telemetry.addData("","");
        if(frontside) telemetry.addData("LIFT", "FRONT");
        else telemetry.addData("LIFT", "SIDE");

        TelemetryPacket packet =  new TelemetryPacket();

        packet.put("fieldCentric", driving.fieldCentric);
        packet.put("righteous", driving.righteous);
        packet.put("speed", driving.speed);

        packet.put("frontside", frontside);

        driving.updateTelemetry(packet);

        dashboard.sendTelemetryPacket(packet);
    }
}