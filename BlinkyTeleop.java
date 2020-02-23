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
    private Gamepad prev1 = new Gamepad();
    private Gamepad prev2 = new Gamepad();
    private int prevSpeed = 0;
    private FailsafeDashboard dashboard = new FailsafeDashboard();
    private Driive driving = new Driive();

    private double angle1 = 0.0;
    private double sideliftspeed = 1;
    private boolean runningside = false;
    private boolean aligning = false;
    private boolean grabbing = false;
    private double grabtime = 0;

    public void init() {
        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double pi = Math.PI;
        double[] angles = {pi/4, pi*3/4, pi*5/4, pi*7/4};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {
            telemetry.addData("Wheels and wheelAngles are not the same length!", "");
            telemetry.update();
        }
        robot.sidelift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sidelift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Initialized", "Initialized");
        telemetry.update();
        driving.speed = 4;
        driving.setMinTurnSpeed(0.08);
        driving.setTurnThreshold(0.05);
    }

    public void init_loop() {
        /*driving.gyro(robot.angles.secondAngle);
        TelemetryPacket packet = new TelemetryPacket();
        driving.updateTelemetry(packet);
        dashboard.sendTelemetryPacket(packet);*/
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition());
    }

    public void loop() {
        // -- DRIVING --
        driving.cartesian(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

        robot.updateGyro();
        double currentAngle = robot.angles.thirdAngle;
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
        if(gamepad1.right_stick_button && !prev1.right_stick_button) {
            prevSpeed = driving.speed;
            driving.speed = 8;
        }
        else if(!gamepad1.right_stick_button && prev1.right_stick_button) {
            driving.speed = prevSpeed;
        }

        try {
            prev1.copy(gamepad1);
        } catch(RobotCoreException e) {}

        // -- SPECIAL MOTORS --

        // Triggers run intake
        if(!robot.intake_button.getState()) {
            robot.rightintake.setPower(0);
            robot.leftintake.setPower(0);
        }
        if(gamepad2.right_trigger - gamepad2.left_trigger <= 0 || robot.intake_button.getState()) {
            robot.rightintake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            robot.leftintake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
        }

        // Lift up/down
        double sideliftpower = gamepad2.left_stick_y * sideliftspeed;
        double sideliftpos = robot.sidelift.getCurrentPosition();
        if(sideliftpower != 0) {
            runningside = false;
            grabbing = false;

            robot.sidelift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // If it's outside the range, don't run
            if((sideliftpos > 0 && sideliftpower > 0) /*|| (sideliftpos < -4500 && sideliftpower < 0)*/) {
                robot.sidelift.setPower(0);
            }
            // Otherwise run
            else robot.sidelift.setPower(sideliftpower);
        }
        // Run all the way down when button is pushed
        else if(gamepad2.left_stick_button) {
            grabbing = false;
            robot.sidelift.setTargetPosition(0);
            robot.sidelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.sidelift.setPower(1);
            runningside = true;
        }
        else if(!runningside) {
            robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition());
            robot.sidelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.sidelift.setPower(1);
        }

        // Side lift speed from A/B
        if(gamepad2.a) sideliftspeed = 1;
        if(gamepad2.b) sideliftspeed = .1;

        // Right stick X controls current lift grabber
        if(gamepad2.right_stick_y < 0) robot.sideliftgrab.setPosition(0);
        else if(gamepad2.right_stick_y > 0) {
            grab();
        }
        if(grabbing) {
            if(getRuntime() - .5 > grabtime) {
                robot.sidelift.setTargetPosition(-300);
                robot.sidelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.sidelift.setPower(1);
                runningside = true;
            }
        }

        // D-pad up/down controls platform grabber
        if(gamepad2.dpad_up) robot.platform.setPosition(0.2);
        else if(gamepad2.dpad_down) robot.platform.setPosition(0.65);

        // Automatic alignment
        double distance = robot.distance_alignment.getDistance(DistanceUnit.CM);
        if(gamepad2.right_stick_button && distance < 3 && robot.sideliftgrab.getPosition() == 0) {
            aligning = true;
        }
        if(aligning) {
            if(distance < 3) {
                boolean prevFC = driving.fieldCentric;
                int prevSpeed = driving.speed;
                driving.fieldCentric = false;
                driving.speed = 10;
                driving.polar(0.1, Math.PI, 0);
                driving.driive();
                driving.fieldCentric = prevFC;
                driving.speed = prevSpeed;
                if(gamepad2.left_bumper || gamepad2.right_bumper) aligning = false;
            }
            else {
                grab();
                driving.polar(0, 0, 0);
                driving.driive();
                aligning = false;
            }
        }

        try {
            prev2.copy(gamepad2);
        } catch(RobotCoreException e) {}

        // -- TELEMETRY --
        telemetry.addData("DRIVING", "");
        telemetry.addData("Field centric", driving.fieldCentric);
        telemetry.addData("Righteous", driving.righteous);
        telemetry.addData("Speed", driving.speed);

        TelemetryPacket packet =  new TelemetryPacket();

        packet.put("fieldCentric", driving.fieldCentric);
        packet.put("righteous", driving.righteous);
        packet.put("speed", driving.speed / 10);

        packet.put("power.sidelift", robot.sidelift.getPower());
        packet.put("position.sidelift", robot.sidelift.getCurrentPosition());
        packet.put("position.one", robot.one.getCurrentPosition());
        packet.put("position.two", robot.two.getCurrentPosition());
        packet.put("position.three", robot.three.getCurrentPosition());
        packet.put("position.four", robot.four.getCurrentPosition());
        packet.put("angles.first", robot.angles.firstAngle);
        packet.put("angles.second", robot.angles.secondAngle);
        packet.put("angles.third", robot.angles.thirdAngle);

        packet.put("distance_alignment", robot.distance_alignment.getDistance(DistanceUnit.CM));
        packet.put("distance_platform", robot.distance_platform.getDistance(DistanceUnit.CM));
        packet.put("distance_unplat", robot.distance_unplat.getDistance(DistanceUnit.CM));
        packet.put("distance_blockplat", robot.distance_blockplat.getDistance(DistanceUnit.CM));
        packet.put("intake_button", robot.intake_button.getState());
        packet.put("battery", robot.getBatteryVoltage());
        packet.put("aligning", aligning);

        driving.updateTelemetry(packet);

        dashboard.sendTelemetryPacket(packet);
    }

    private void grab() {
        robot.sideliftgrab.setPosition(0.8);
        grabbing = true;
        grabtime = getRuntime();
    }
}
