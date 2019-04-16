$fn = 50;

use <MCAD/2Dshapes.scad>
use <../../lib/pcb_led_cvf.scad>
use <../../lib/ear.scad>

wall = 2;
hole_d = 3.2;
hole_d_through = 4;

led_count_x = 10;
led_count_y = 10;

led_d = 5.5;
led_leg_sz = 16;
led_holder_sz_z = min(4, led_leg_sz - 7);

led_step_x = led_d + 1.5;
led_step_y = led_step_x;

led_leg_encircle = 3.1;
led_holder_wall = 1;

holder_skirt_sz = 8;
led_holder_plate_sz_x = led_step_x * led_count_x + 2 * holder_skirt_sz;
led_holder_plate_sz_y = led_step_y * led_count_y + 2 * holder_skirt_sz;
led_holder_plate_sz_z = wall;

stepdown_pcb_sz_x = 48.1;
stepdown_pcb_sz_y = 24;
stepdown_pcb_sz_z = 1.1;

stepdown_holder_sz_x = led_holder_plate_sz_x - 2 * holder_skirt_sz - 2 * wall;
stepdown_holder_sz_y = led_holder_plate_sz_y - 2 * holder_skirt_sz - 2 * wall;
stepdown_holder_sz_z = 20;

power12_d = 8.4;

acryl_sz_x = 90;
acryl_sz_y = 118; //?
acryl_sz_z = 3.2;

acryl_gap = 0.5;
acryl_holder_sz_x = max(led_holder_plate_sz_x, acryl_sz_x + acryl_gap);
acryl_holder_sz_y = max(led_holder_plate_sz_y, acryl_sz_y + acryl_gap);
acryl_holder_sz_z = 30;

module a_led_holder(with_hole=true) {
	difference() {
		if(with_hole) {
			cylinder(d=led_leg_encircle+2*led_holder_wall, h=led_holder_sz_z, center=true);
		}
		difference() {
			cylinder(d=led_leg_encircle, h=led_holder_sz_z, center=true);
			cube([1, led_leg_encircle+2*led_holder_wall, led_holder_sz_z], center=true);
		}
	}
}

module switch_holes() {
	cube([9.5, 5, 50], center=true);
	translate([9.5, 0, 0]) {
		cylinder(d=2.5, h = 50, center=true);
	}
	translate([-9.5, 0, 0]) {
		cylinder(d=2.5, h = 50, center=true);
	}
}

module ear(base_dist, height) {
	bolt_hole_cone_center(base_dist=base_dist, hole_d=3, hole_wall=3, height=height, hole_height=height);
}

module ear_holes(d, h=50, base_dist=3, ear_mode=false, height=7) {
	for(xsgn=[-1, 1]) {
		for(ysgn=[-1, 1]) {
			if(abs(xsgn)==1 || abs(ysgn)==1) {
				translate([xsgn*(led_holder_plate_sz_x-holder_skirt_sz)/2, ysgn*(led_holder_plate_sz_y-holder_skirt_sz)/2, 0]) {
					if(ear_mode) {
						if(abs(xsgn) == 1) {
							angle = xsgn > 0 ? 0 : 180;
							rotate([180, 0, angle]) {
								ear(base_dist=base_dist, height=height);
							}
						} else {
							if(abs(ysgn) != 0) {
								angle = ysgn > 0 ? 90 : -90;
								rotate([180, 0, angle]) {
									ear(base_dist=base_dist, height=height);
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

module led_holder_plate() {
	difference() {
		union() {
			cube([led_holder_plate_sz_x, led_holder_plate_sz_y, led_holder_plate_sz_z], center=true);
			for(x=[1:led_count_x]) {
				for(y=[1:led_count_y]) {
					translate([-led_holder_plate_sz_x/2+x*led_step_x+led_step_x/2, -led_holder_plate_sz_y/2+y*led_step_y+led_step_y/2, (led_holder_sz_z+led_holder_plate_sz_z)/2]) {
						a_led_holder(with_hole=true);
					}
				}
			}
		}
		for(x=[0:led_count_x-1]) {
			for(y=[0:led_count_y-1]) {
				translate([-led_holder_plate_sz_x/2+x*led_step_x+led_step_x/2, -led_holder_plate_sz_y/2+y*led_step_y+led_step_y/2, (led_holder_sz_z+led_holder_plate_sz_z)/2]) {
					a_led_holder(with_hole=false);
				}
			}
		}
		ear_holes(d=hole_d);
	}
}

module stepdown_holder() {
	difference() {
		union() {
			for(ysgn=[-1,1]) {
				// long ends
				translate([0, ysgn*(stepdown_holder_sz_y+wall)/2, 0]) {
					cube([stepdown_holder_sz_x, wall, stepdown_holder_sz_z], center=true);
					translate([0, ysgn*(holder_skirt_sz+wall)/2, (stepdown_holder_sz_z-wall)/2]) {
						cube([stepdown_holder_sz_x+2*wall, holder_skirt_sz, wall], center=true);
					}
				}
			}
			for(xsgn=[-1,1]) {
				// short ends
				translate([xsgn*(stepdown_holder_sz_x+wall)/2, 0, 0]) {
					cube([wall, stepdown_holder_sz_y+2*wall, stepdown_holder_sz_z], center=true);
					translate([xsgn*(holder_skirt_sz+wall)/2, 0, (stepdown_holder_sz_z-wall)/2]) {
						cube([holder_skirt_sz, 2*holder_skirt_sz+stepdown_holder_sz_y+2*wall, wall], center=true);
					}
				}
			}
			translate([0, 0, -stepdown_holder_sz_z/2]) {
				cube([stepdown_holder_sz_x+2*wall, stepdown_holder_sz_y+2*wall, wall], center=true);
			}
			translate([-25, -40 , -stepdown_holder_sz_z/2]) {
				pcb_led_cvf(
					pcb_width = stepdown_pcb_sz_x,
					pcb_length = stepdown_pcb_sz_y,
					pcb_height = stepdown_pcb_sz_z,
					add_tabs = false
				);
			}
		}
		ear_holes(d=hole_d_through);
		// 12v power hole
		translate([-19, stepdown_holder_sz_y-25, 0]) {
			rotate([90, 0, 0]) {
				cylinder(d=power12_d, h=50);
			}
		}
		translate([10, stepdown_holder_sz_y-50, 0]) {
			rotate([90, 0, 0]) {
				switch_holes();
			}
		}
	}
}

module acryl_holder() {
	difference() {
		union() {
			for(ysgn=[-1,1]) {
				// long ends
				translate([0, ysgn*(acryl_holder_sz_y+wall)/2, 0]) {
					cube([acryl_holder_sz_x+2*wall, wall, acryl_holder_sz_z], center=true);
					translate([0, -ysgn*wall, -acryl_sz_z/2]) {
						cube([acryl_holder_sz_x+2*wall, wall, acryl_holder_sz_z-acryl_sz_z], center=true);
					}
					translate([0, -ysgn*(acryl_holder_sz_y-stepdown_holder_sz_y)/4, -(acryl_holder_sz_z-wall)/2]) {
						cube([acryl_holder_sz_x+2*wall, (acryl_holder_sz_y-stepdown_holder_sz_y)/2, wall], center=true);
					}
				}
			}
			for(xsgn=[-1,1]) {
				// short ends
				translate([xsgn*(acryl_holder_sz_x+wall)/2, 0, 0]) {
					cube([wall, acryl_holder_sz_y+2*wall, acryl_holder_sz_z], center=true);
					translate([-xsgn*wall, 0, -acryl_sz_z/2]) {
						cube([wall, acryl_holder_sz_y+2*wall, acryl_holder_sz_z-acryl_sz_z], center=true);
					}
					translate([-xsgn*(acryl_holder_sz_x-stepdown_holder_sz_x)/4, 0, -(acryl_holder_sz_z-wall)/2]) {
						cube([(acryl_holder_sz_x-stepdown_holder_sz_x)/2, acryl_holder_sz_y+2*wall, wall], center=true);
					}
				}
			}
			translate([0, 0, (acryl_holder_sz_z-acryl_sz_z)/2]) {
				%cube([acryl_sz_x, acryl_sz_y, acryl_sz_z], center=true);
			}
		}
		ear_holes(d=hole_d);
	}
	translate([0, 0, -(acryl_holder_sz_z-acryl_sz_z)/2]) {
		ear_holes(d=hole_d, ear_mode=true, height=acryl_holder_sz_z-acryl_sz_z-wall);
	}
}

module diffuser(x_count, y_count, step_x, step_y) {
	sz_x = x_count*step_x;
	sz_y = y_count*step_y;
	sz_z = max(step_x, step_y);
	difference() {
		union() {
			translate([-sz_x/2, -sz_y/2, 0]) {
				for(x=[0:x_count]) {
					for(y=[0:y_count]) {
						translate([x*step_x, y*step_y, 0]) {
							sphere(d=max(step_x, step_y), center=true);
						}
					}
				}
			}
			translate([0, 0, 0.5]) {
				cube([sz_x, sz_y, 1], center=true);
			}
		}
		translate([0, 0, -sz_z/2]) {
			cube([sz_x*2, sz_y*2, sz_z], center=true);
		}
	}
}

module led_diffuser() {
	$fn = 10;
	d = max(led_step_x, led_step_y);
	scale([0.99, 0.99, 1]) {
		difference() {
				cube([acryl_holder_sz_x-2*wall, acryl_holder_sz_y-2*wall, 1], center=true);
				translate([0, 0, -(acryl_holder_sz_z-acryl_sz_z)/2]) {
					ear_holes(d=hole_d);
					ear_holes(d=hole_d, ear_mode=true, height=acryl_holder_sz_z-acryl_sz_z-wall);
				}
		}
	}
	for(ygs1=[-1,1]) {
		translate([0, ygs1*(pcb_sz_y-wall)/2, 0]) {

//			cube([pcb_sz_x-wall, pcb_sz_y-wall, 1], center=true);
			for(x=[0:led_count_x-1]) {
				for(y=[0:led_count_y-1]) {
					translate([pcb_first_hole_x+x*led_step_x-pcb_sz_x/2+pcb_hole_dist/2, pcb_first_hole_y+y*led_step_y-pcb_sz_y/2+pcb_hole_dist, 0]) {
						difference() {
							scale([1, 1, 0.6]) {
								sphere(d=d, center=true);
							}
							translate([0, 0, -d/2]) {
								cube([d, d, d], center=true);
							}
						}
					}
				}
			}
		}
	}
}

//diffuser(10, 10, pcb_hole_dist, pcb_hole_dist);
translate([0, 0, acryl_holder_sz_z]) {
//union() {
difference() {
//	led_diffuser();
	translate([40, 0, 0]) {
//		cube([100, 100, 100], center=true);
	}
	translate([0, 40, 0]) {
//		cube([100, 100, 100], center=true);
	}
}
}

translate([0, 0, acryl_holder_sz_z]) {
//	acryl_holder();
}
led_holder_plate();
translate([0, 0, -stepdown_holder_sz_z]) {
//	stepdown_holder();
}
