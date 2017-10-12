Header header

#from gearbox
uint32 gear				    #done
string gearbox_status	    #pending

#from plc
float64 clutch			    #done
float64 brake			    #done
float64 steering_wheel  	#done
float64 rpm				    #done

uint32 lights_brake		    #pending
uint32 lights_reverse	    #pending
uint32 lights_warning	    #pending

uint32 emergency		    #done

uint32 auto_ignition	    #done
uint32 auto_brake		    #done
uint32 auto_direction	    #done
uint32 auto_clutch		    #done

#from throttle
float64 throttle		    #pending
float64 throttle_pedal	    #done
uint32 auto_throttle	    #done

#from odometry
float64 velocity		    #done
float64 count              #done
float64 pulses_sec         #done
float64 revolutions_sec    #done


#from driver
uint32 lights_high		    #done
uint32 lights_medium	    #done
uint32 lights_left		    #done
uint32 lights_right		    #done
uint32 lights_danger	    #done

uint32 ignition			    #done

uint32 horn				    #pending

uint32 throttle_pressure	#done
uint32 brake_pressure		#done
uint32 clutch_pressure		#done