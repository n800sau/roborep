$fn = 50;

use <MCAD/2Dshapes.scad>


wall = 2;
hole_d = 3.2;

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
//pcb_first_hole_x = 0;
//pcb_first_hole_y = 0;
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

pcb_inter_gap = 0.5;
full_size_pcb_holder_plate_sz_y = 2 * pcb_sz_y + pcb_inter_gap + 2 * holder_skirt_sz;

stepdown_holder_sz_z = 15; //?

module a_led_holder(with_hole=true) {
	difference() {
		if(with_hole) {
			cylinder(d=led_leg_encircle+2*led_holder_wall, h=led_holder_sz_z, center=true);
		}
		cylinder(d=led_leg_encircle, h=led_holder_sz_z, center=true);
	}
}

module sandwich_holes(d, h, arc=false, arc_start=0, arc_end=180) {
	// sandwich holes
	for(xsgn=[-1, 0, 1]) {
		for(ysgn=[-1, -0.5, 0, 0.5, 1]) {
			if(abs(xsgn)==1 || abs(ysgn)==1) {
				translate([xsgn*(pcb_holder_plate_sz_x-holder_skirt_sz)/2, ysgn*(full_size_pcb_holder_plate_sz_y-holder_skirt_sz)/2, 0]) {
					if(arc) {
						translate([0, 0, -h/2]) {
							if(abs(xsgn) == 1) {
								linear_extrude(height = h) {
									$angle = xsgn > 0 ? 90 : -90;
									donutSlice(d-2*wall, d, $angle, $angle+180);
								}
							}
							if(abs(ysgn) != 0) {
								linear_extrude(height = h) {
									$angle = ysgn > 0 ? 180 : 0;
									donutSlice(d-2*wall, d, $angle, $angle+180);
								}
							}
						}
					} else {
						cylinder(d=d, h=h, center=true);
					}
				}
			}
		}
	}
}

module pcb_holder_plate() {
	difference() {
		for(ygs1=[-1,1]) {
			translate([0, ygs1*(pcb_sz_y/2+0.25), 0]) {
				cube([pcb_holder_plate_sz_x, pcb_holder_plate_sz_y, pcb_holder_plate_sz_z], center=true);
				for(x=[0:led_count_x-1]) {
					for(y=[0:led_count_y-1]) {
						translate([pcb_first_hole_x+x*led_step_x-pcb_sz_x/2+pcb_hole_dist/2, pcb_first_hole_y+y*led_step_y-pcb_sz_y/2+pcb_hole_dist, (led_holder_sz_z+pcb_holder_plate_sz_z)/2]) {
							a_led_holder(with_hole=true);
						}
					}
				}
			}
		}
		for(ygs2=[-1,1]) {
			translate([0, ygs2*(pcb_sz_y/2+0.25), 0]) {
				// holes in pcb area (for preview only)
				%for(x=[0:hole_count_x-1]) {
					for(y=[0:hole_count_y-1]) {
						translate([pcb_first_hole_x+x*pcb_hole_dist-pcb_sz_x/2, pcb_first_hole_y+y*pcb_hole_dist-pcb_sz_y/2, 0]) {
							cylinder(d=2, h=50, center=true);
						}
					}
				}
				for(x=[0:led_count_x-1]) {
					for(y=[0:led_count_y-1]) {
						translate([pcb_first_hole_x+x*led_step_x-pcb_sz_x/2+pcb_hole_dist/2, pcb_first_hole_y+y*led_step_y-pcb_sz_y/2+pcb_hole_dist, 0]) {
							a_led_holder(with_hole=false);
						}
					}
				}
				// sandwich holes
				for(xsgn=[-1,0,1]) {
					for(ysgn=[-1,0,1]) {
						if(xsgn!=0 || (ysgn==-1 && ygs2==-1) || (ysgn==1 && ygs2==1)) {
							translate([xsgn*(pcb_sz_x+holder_skirt_sz)/2, ysgn*(pcb_sz_y+holder_skirt_sz)/2, 0]) {
//								cylinder(d=hole_d, h=50, center=true);
							}
						}
					}
				}
				// holes in pcb
				for(xsgn=[-1,1]) {
					for(ysgn=[-1,1]) {
						translate([xsgn*hole_dist_x/2, ysgn*hole_dist_y/2, 0]) {
							cylinder(d=2, h=50, center=true);
						}
					}
				}
			}
		}
		sandwich_holes(d=hole_d, h=50);
	}
}

module stepdown_holder() {
	difference() {
		union() {
			for(ysgn=[-1,1]) {
				translate([0, ysgn*full_size_pcb_holder_plate_sz_y/2, 0]) {
					cube([pcb_holder_plate_sz_x+2*wall, wall, stepdown_holder_sz_z], center=true);
				}
			}
			for(xsgn=[-1,1]) {
				translate([xsgn*pcb_holder_plate_sz_x/2, 0, 0]) {
					cube([wall, full_size_pcb_holder_plate_sz_y+2*wall, stepdown_holder_sz_z], center=true);
				}
			}
			translate([0, 0, -stepdown_holder_sz_z/2]) {
				cube([pcb_holder_plate_sz_x+2*wall, full_size_pcb_holder_plate_sz_y+2*wall, wall], center=true);
			}
		}
		sandwich_holes(d=hole_d+wall*4, h=stepdown_holder_sz_z);
	}
	sandwich_holes(d=hole_d+wall*4, h=stepdown_holder_sz_z, arc=true, arc_start=0, arc_end=180);
}

//pcb_holder_plate();
translate([0, 0, -stepdown_holder_sz_z]) {
	stepdown_holder();
}
