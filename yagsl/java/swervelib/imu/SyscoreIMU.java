package swervelib.imu;

import java.util.Optional;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;

public class SyscoreIMU extends SwerveIMU {
    private OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);

    private Rotation3d offset = Rotation3d.kZero;

    @Override
    public void close() {}

    @Override
    public void factoryDefault() {
        offset = Rotation3d.kZero;
    }

    @Override
    public void clearStickyFaults() {
    }

    @Override
    public void setOffset(Rotation3d offset) {
        // TODO: Not sure if inverse() is correct here
        this.offset = offset.inverse();
    }

    @Override
    public void setInverted(boolean invertIMU) {
        throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
    }

    @Override
    public Rotation3d getRawRotation3d() {
        return imu.getRotation3d();
    }

    @Override
    public Rotation3d getRotation3d() {
        return imu.getRotation3d().rotateBy(offset);
    }

    @Override
    public Optional<Translation3d> getAccel() {
        return Optional.of(new Translation3d(
            imu.getAccelX(),
            imu.getAccelY(),
            imu.getAccelZ()
        ));
    }

    @Override
    public AngularVelocity getYawAngularVelocity() {
        // TODO: Verify Y is the correct axis
        return Units.RadiansPerSecond.of(imu.getGyroRateY());
    }

    @Override
    public Object getIMU() {
        return imu;
    }

}
