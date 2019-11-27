$fn = 30;

wall = 2;
top_d = 15;

module servo_hand() {
	cylinder(d=top_d+2*wall, h=wall);
	
}


servo_hand();
