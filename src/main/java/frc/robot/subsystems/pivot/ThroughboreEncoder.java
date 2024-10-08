package frc.robot.subsystems.pivot;

import com.revrobotics.SparkAbsoluteEncoder;

public class ThroughboreEncoder {
    SparkAbsoluteEncoder abs;
    double offset;

    public ThroughboreEncoder(SparkAbsoluteEncoder absoluteEncoder, double absOffset) {
        abs = absoluteEncoder;
        offset = absOffset;
    }

    public double getPosition() {
        return (abs.getPosition() - offset) % (2 * Math.PI) + ((abs.getPosition() + offset) < 0 ? 2 * Math.PI : 0);
    }

    public void setOffset(double absOffset) {
        offset = absOffset;
    }

    public double getOffset() {
        return offset;
    }
}
