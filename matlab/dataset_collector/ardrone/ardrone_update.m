function ardrone_update( )
    global ardrone

    cmd = sprintf('DRIVE {AltitudeVelocity %f} {LinearVelocity %f} {LateralVelocity %f} {RotationalVelocity %f} {Normalized false}', ardrone.AltitudeVelocity, ardrone.LinearVelocity, ardrone.LateralVelocity, ardrone.RotationalVelocity);
    
    interface_send(cmd);

end

