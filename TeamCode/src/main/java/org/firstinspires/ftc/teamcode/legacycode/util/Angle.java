package org.firstinspires.ftc.teamcode.legacycode.util;

/*
idk what to do with this
i'm not sure this is the right solution to the angle problem
maybe just enforce all angles to be radians from -pi to pi

also i'm not sure if we should have fields for the bounds of the angle
it would probably make things less messy
but this really shouldn't be a super complicated ordeal

i think maybe the best course of action is just to make sure all angles are a certain format
because you can't really tell if the new value being set is radians or degrees here
so that's a big issue
and also you can't tell what other screw ups the user may have
 */
public class Angle {
    public double value;
    protected Format format;

    public enum Format {
        RADIANS,
        DEGREES
    }

    public Angle(double value, Format format) {
        this.value = value;
        this.format = format;
    }

    public Angle set(Format newFormat) {
        if (format != newFormat) {
            format = newFormat;
            value = newFormat == Format.RADIANS ? Math.toRadians(value) : Math.toDegrees(value);
        }
        return this;
    }
    public Angle set(double value, Format newFormat) {
        return this;
    }

    /**
     * Sets the Angle's {@code value} to within the specified bounds. We only take then lower bound
     * because the upper bound is <b>always</b> a full circle (360 deg or 2 pi rad) greater.
     *
     * @param lowerBound the lower bound in current {@code format} to set the angle
     * @return the {@code value} of the Angle after transformation
     */
    public double setBounds(double lowerBound) {
        double circle = format == Format.DEGREES ? 360 : 2 * Math.PI;
        while (value <= lowerBound) {
            value += circle;
        }
        while (value > lowerBound + circle) {
            value -= circle;
        }
        return value;
    }
}
