package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

class FailsafeDashboard {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    void sendTelemetryPacket(TelemetryPacket packet) {
        if(dashboard != null) {
            dashboard.sendTelemetryPacket(packet);
        }
    }

    void startCameraStream(VuforiaLocalizer source, int maxFps) {
        if(dashboard != null) {
            dashboard.startCameraStream(source, maxFps);
        }
    }
}
