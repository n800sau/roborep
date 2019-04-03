$fn = 50;

wall = 2;

hole_dist_x = 64;
hole_dist_y = 44.5;

hole_count_x = 24;
hole_count_y = 18;

pcb_sz_x = 70;
pcb_sz_y = 50.5;
pcb_sz_z = 1.2;
pcb_hole_dist = 2.54;
pcb_first_hole_x = (pcb_sz_x - pcb_hole_dist * (hole_count_x-1))/2;
pcb_first_hole_y = (pcb_sz_y - pcb_hole_dist * (hole_count_y-1))/2;
//echo("pcb_first_hole_x=", pcb_first_hole_x);
//echo("pcb_first_hole_y=", pcb_first_hole_y);

hole_per_led_x = 3;
hole_per_led_y = 3;

led_count_x = hole_count_x / hole_per_led_x;
led_count_y = hole_count_y / hole_per_led_y;

led_step_x = pcb_hole_dist * hole_per_led_x;
led_step_y = pcb_hole_dist * hole_per_led_y;

led_d = 5.5;
led_leg_sz = 16;
led_holder_sz_z = min(4, led_leg_sz - pcb_sz_z - wall - 4);

//echo("led_holder_sz_z=", led_holder_sz_z);

led_leg_encircle = 3.1;
led_holder_wall = 1;

holder_skirt_sz = 8;
pcb_holder_plate_sz_x = pcb_sz_x + 2 * holder_skirt_sz;
pcb_holder_plate_sz_y = pcb_sz_y + 2 * holder_skirt_sz;
pcb_holder_plate_sz_z = wall;


module a_led_holder(with_hole=true) {
	difference() {
		if(with_hole) {
			cylinder(d=led_d+2*led_holder_wall, h=led_holder_sz_z, center=true);
		}
		cylinder(d=led_d, h=led_holder_sz_z, center=true);
	}
}

module pcb_holder_plate() {
	difference() {
		cube([pcb_holder_plate_sz_x, pcb_holder_plate_sz_y, pcb_holder_plate_sz_z], center=true);
		for(x=[0:led_count_x]) {
			for(y=[0:led_count_y]) {
				translate([pcb_first_hole_x+x*led_step_x-pcb_sz_x/2, pcb_first_hole_y+y*led_step_y-pcb_sz_y/2, 0]) {
					a_led_holder(with_hole=false);
				}
			}
		}
	}
		for(xsgn=[-1,1]) {
			for(ysgn=[-1,1]) {
				translate([xsgn*hole_dist_x/2, ysgn*hole_dist_y/2, 0]) {
					cylinder(d=2, h=50, center=true);
				}
			}
		}
	for(x=[0:led_count_x]) {
		for(y=[0:led_count_y]) {
			translate([pcb_first_hole_x+x*led_step_x-pcb_sz_x/2, pcb_first_hole_y+y*led_step_y-pcb_sz_y/2, (led_holder_sz_z+pcb_holder_plate_sz_z)/2]) {
				a_led_holder(with_hole=true);
			}
		}
	}
}

module double_pcb_holder_plate() {
	translate([0, -pcb_sz_y/2-1, 0]) {
		pcb_holder_plate();
	}
	translate([0, pcb_sz_y/2+1, 0]) {
		pcb_holder_plate();
	}
}



double_pcb_holder_plate();
