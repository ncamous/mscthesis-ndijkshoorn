function ardrone_set ( param, change )

    global ardrone;
    
    ardrone.(param) = ardrone.(param) + change;
    
    ardrone_update ();

end

