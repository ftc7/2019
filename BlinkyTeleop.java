package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Blinky")
public class BlinkyTeleop extends OpMode {
    private Blinky robot = new Blinky();
    //private gpPrev1 prev1 = new gpPrev1();
    private Gamepad prev1 = new Gamepad();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Driive driving = new Driive();

    private double angle1 = 0.0;
    private boolean frontside = false;
    private double sideliftspeed = 1;
    private boolean runningsidedown = false;
    private boolean aligning = false;

    public void init() {
        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double pi = Math.PI;
        double[] angles = {pi/4, pi*3/4, pi*5/4, pi*7/4};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {
        }
        robot.sidelift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sidelift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if(!aligning) driving.driive();

        // -- CONTROLS --

        if(gamepad1.left_bumper) angle1 = currentAngle;
        if(gamepad1.right_bumper) driving.turnAbs(angle1);
        if(gamepad1.dpad_up) driving.turnAbs(0);
        else if(gamepad1.dpad_down) driving.turnAbs(Math.PI);
        else if(gamepad1.dpad_left) driving.turnAbs(Math.PI / 2);
        else if(gamepad1.dpad_right) driving.turnAbs(Math.PI * 3 / 2);

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
        double sideliftpower = gamepad2.left_stick_y * sideliftspeed;
        double sideliftpos = robot.sidelift.getCurrentPosition();
        // Front lift
        if(frontside) {
            if((robot.frontlift.getCurrentPosition() > 0 && gamepad2.left_stick_y > 0) ||
                    (robot.frontlift.getCurrentPosition() < -4000 && gamepad2.left_stick_y < 0)) {
                robot.frontlift.setPower(0);
            }
            else robot.frontlift.setPower(gamepad2.left_stick_y / 6);
        }
        // Side lift
        else {
            // If the joystick is being used
            if(sideliftpower != 0) {
                runningsidedown = false;

                robot.sidelift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // If it's outside the range, don't run
                if((sideliftpos > 0 && sideliftpower > 0) /*|| (sideliftpos < -4000 && sideliftpower < 0)*/) {
                    robot.sidelift.setPower(0);
                }
                // Otherwise run
                else robot.sidelift.setPower(sideliftpower);
            }
            else if(gamepad2.left_stick_button) {
                robot.sidelift.setTargetPosition(0);
                robot.sidelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.sidelift.setPower(1);
                runningsidedown = true;
            }
            else if(!runningsidedown) {
                robot.sidelift.setPower(0);
            }
        }

        // Side lift speed from A/B
        if(gamepad2.a) sideliftspeed = 1;
        if(gamepad2.b) sideliftspeed = .1;

        // Right stick X controls current lift grabber
        if(frontside) robot.frontliftgrab.setPower(gamepad2.right_stick_x);
        else if(gamepad2.right_stick_y < 0) robot.sideliftgrab.setPosition(0);
        else if(gamepad2.right_stick_y > 0) robot.sideliftgrab.setPosition(0.6);

        // D-pad up/down controls platform grabber
        if(gamepad2.dpad_up) robot.platform.setPosition(0.2);
        else if(gamepad2.dpad_down) robot.platform.setPosition(0.65);

        // Automatic alignment
        double distance = robot.distance.getDistance(DistanceUnit.CM);
        if(gamepad2.right_stick_button && !frontside && distance < 3 && robot.sideliftgrab.getPosition() == 0) {
            aligning = true;
        }
        if(aligning) {
            if(distance < 3) {
                boolean prevFC = driving.fieldCentric;
                driving.fieldCentric = false;
                driving.polar(0.2, Math.PI, 0);
                driving.driive();
                driving.fieldCentric = prevFC;
            }
            else {
                robot.sideliftgrab.setPosition(0.6);
                driving.polar(0, 0, 0);
                driving.driive();
                aligning = false;
            }
        }

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


        packet.put("power.frontlift", robot.frontlift.getPower());
        packet.put("position.frontlift", robot.frontlift.getCurrentPosition());
        packet.put("power.sidelift", robot.sidelift.getPower());
        packet.put("position.sidelift", robot.sidelift.getCurrentPosition());
        packet.put("power.track", robot.track.getPower());
        packet.put("position.track", robot.track.getCurrentPosition());
        //packet.put("one.power", robot.one.getPower());
        packet.put("position.one", robot.one.getCurrentPosition());
        //packet.put("two.power", robot.two.getPower());
        packet.put("position.two", robot.two.getCurrentPosition());
        //packet.put("three.power", robot.three.getPower());
        packet.put("position.three", robot.three.getCurrentPosition());
        //packet.put("four.power", robot.four.getPower());
        packet.put("position.four", robot.four.getCurrentPosition());

        packet.put("distance", robot.distance.getDistance(DistanceUnit.CM));
        packet.put("aligning", aligning);

        driving.updateTelemetry(packet);

        dashboard.sendTelemetryPacket(packet);
    }
}
