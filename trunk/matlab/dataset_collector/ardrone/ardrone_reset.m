function ardrone_reset( input_args )
    global ardrone;

    ardrone.AltitudeVelocity = 0.0;
    ardrone.LinearVelocity = 0.0;
    ardrone.LateralVelocity = 0.0;
    ardrone.RotationalVelocity = 0.0;

    ardrone_update ();

end

