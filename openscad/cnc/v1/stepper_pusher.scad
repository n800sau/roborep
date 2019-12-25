include <cnc_params.scad>
use <MCAD/regular_shapes.scad>
use <../../lib/Getriebe.scad>

$fn = 50;

module pusher_frame() {
	f_size = bar_side_length;
	//f_size = 10;
	difference() {
		union() {
			translate([-bar_length/2, pusher_height+bar_thick, -bar_side_length/2]) {
				cube([bar_side_side_width+10.1, 4, f_size]);
				translate([bar_side_side_width+10.1, -2, 0]) {
					cube([5, pusher_width, f_size]);
				} 
			}
		}
		side_holes();
	}
}

module pusher() {
	p_size = bar_side_length + 50;
	//p_size = 10;
	difference() {
//	union() {
		union() {
			translate([-pusher_width/2+9.6, 10.2, 1]) {
				rotate([0, -90, 90]) {
					zahnstange(modul=0.9, laenge=p_size, hoehe=19.7, breite=pusher_width, eingriffswinkel=25, schraegungswinkel=0); 
				}
			}
			translate([16.4, bar_thick, -p_size/2]) {
				cube([10, pusher_height-0.4, p_size]);
			}
		}
		translate([16.4+5, bar_thick+pusher_height/2, -p_size/2-5]) {
			cylinder(d=hole_d_m3, h=p_size+10);
		}
	}
}

module stepper_frame() {
	difference() {
//	union() {
		translate([-bar_length/2, 0, -bar_side_length/2]) {
			union() {
				// top bottom bars
				for(z=[0,1]) {
					translate([0, 0, z*stepper_hole_dist]) {
						cube([bar_length, bar_thick, bar_width]);
					}
				}
				// left right bars
				for(x=[0,1]) {
						union() {
							translate([x*(bar_length-bar_side_width), 0, 0]) {
								cube([bar_side_width, bar_thick, bar_side_length]);
							}
							translate([x*(bar_length-bar_side_side_width), 0, 0]) {
								cube([bar_side_side_width, bar_side_side_thick, bar_side_length]);
							}
						}
				}
			}
		}
		side_holes();
		translate([0, 50, -stepper_hole_dist/2]) {
			for(z=[0,1]) {
				translate([0, 0, z*stepper_hole_dist]) {
					rotate([90, 0, 0]) {
						cylinder(d=hole_d_through_m3, h=100);
					}
				}
			}
		}
	}
}

module side_holes() {
	translate([-bar_length/2, 0, -bar_side_length/2]) {
		for(x=[0,1]) {
			n_holes = 5;
			h_step = bar_side_length / (n_holes + 1);
			translate([x*(bar_length-bar_side_side_width), 0, h_step/2]) {
				for(z=[0:1:n_holes]) {
					translate([bar_side_side_width/2, 50, z*h_step]) {
						rotate([90, 0, 0]) {
							cylinder(d=hole_d_through_m3, h=100);
						}
					}
				}
			}
		}
	}
}

module stepper_mockup() {
	translate([0, -15, 0]) {
		rotate([-90, 0, 0]) {
			cylinder(d=stepper_body_d, h=15);
			cylinder(d=gear_d, h=25);
		}
	}
}

pusher_frame();
rotate([0, 180, 0]) {
	%pusher_frame();
}
%pusher();
//color("blue")
	rotate([0, 180, 0]) {
%		pusher();
	}


%stepper_frame();
%stepper_mockup();
