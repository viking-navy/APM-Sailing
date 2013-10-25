APM-Sailing
===========

I need a sailing autopilot for my 3 foot Viking ship.  This looks like a good place to build the code.



Here is what I put in the "one second loop".  Just to see what data I could find.

hal.console->print("\n\n .................................... ");
    
         hal.console->print(" roll ");   /// pete test
         hal.console->print(ahrs.roll_sensor); 
         hal.console->print("  pitch  ");
         hal.console->print(ahrs.pitch_sensor); 
         
          hal.console->print("  pete  \n ");
           hal.console->print("\n\n .................................... ");
    
         hal.console->print("  longitude  ");  // lat and lon seem to work fine
         hal.console->print(g_gps->longitude); 
         hal.console->print(" latitude  ");
         hal.console->print(g_gps->latitude); 
     
      hal.console->print("  pete  \n ");
       hal.console->print("\n\n .................................... ");
    
        hal.console->print("  channel_rudder->control_in  ");
         hal.console->print(channel_rudder->control_in ); 
          hal.console->print("  channel_roll->control_in  ");
         hal.console->print(channel_roll->control_in ); 
           hal.console->print("  channel_pitch->control_in  ");
         hal.console->print(channel_pitch->control_in ); 
          hal.console->print("  channel_throttle->control_in  ");
         hal.console->print(channel_throttle->control_in ); 
      hal.console->print("  pete  \n ");
    hal.console->print("\n\n .................................... ");
    
     
      hal.console->print("  next_WP.lat  ");
      hal.console->print( next_WP.lat ); 
        hal.console->print("  next_WP.lng  ");
      hal.console->print( next_WP.lng ); 
       
         hal.console->print("  pete  \n ");
 //////////////////////        //////////////////////// old_target_bearing_cd (no good)
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
