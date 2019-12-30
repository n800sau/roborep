include <cnc_params.scad>

show_part_1 = 1;
show_part_2 = 1;
show_part_3 = 1;


$fn = 40;

circle_hole_h = 4.5;
top_wall = 4;
circle_wall = 1.5;
wall = 2;


top_d = 19.5;
sensor_d = 7.9;
sensor_tail_width = 7;
sensor_tail_height = 0.4;
bolt_d = 3;
bolt_d_through = 3.6;
bolt_head_h = 2.8;
bolt_head_d = 5.7;

servo_pole_h = 30;
servo_pole_w = sensor_tail_width;
servo_pole_len = top_d/2+wall;
servo_pole_extra_len = 5;
servo_pole_out_len = 5;
servo_pole_hole_off = 5;

pole_gap = 0.4;

servo_handle_len = 40;
servo_handle_h = 8;
servo_handle_w = servo_pole_w+2*wall+2*pole_gap;
servo_hole_d = 5.2;
servo_hole_sz = 3;
servo_bolt_d = 3.6;

module pipette_pusher(do_holes=true) {
	difference() {
		cylinder(d=top_d+2*circle_wall, h=top_wall+circle_hole_h);
		if(do_holes) {
			for(yp=[0, 6]) {
				translate([0, yp, 0]) {
					cylinder(d=bolt_d_through, h=top_wall);
					translate([0, 0, top_wall-bolt_head_h]) {
						cylinder(d=bolt_head_d, h=top_wall+2);
					}
				}
			}
		}
		translate([0, 0, top_wall]) {
			cylinder(d=top_d, h=circle_hole_h);
			translate([-sensor_tail_width/2, 0, 0]) {
				cube([sensor_tail_width, top_d/2+wall, wall]);
			}
		}
	}
}

module servo_pole() {
//	union() {
	difference() {
		translate([0, -servo_pole_extra_len, 0]) {
			intersection() {
//			union() {
				cube([servo_pole_w, servo_pole_len+servo_pole_extra_len+servo_pole_out_len, servo_pole_h]);
				translate([0, servo_pole_len/2, servo_pole_h/2]) {
					rotate([0, 90, 0]) {
						cylinder(d=servo_pole_h+7, h=servo_pole_w);
					}
				}
			}
		}
		translate([servo_pole_w/2, 0, servo_handle_h]) {
			for(yp=[0, 6]) {
				translate([0, yp, 0]) {
					cylinder(d=bolt_d, h=servo_pole_h);
				}
			}
		}
		for(p=[[0, 0, bolt_d_through], [12, 0, bolt_d], [12, 10, bolt_d], [12, 20, bolt_d]]) {
			translate([-10, p[0], p[1]+servo_pole_hole_off]) {
				rotate([0, 90, 0]) {
					cylinder(d=p[2], h=30);
				}
			}
		}
	}
}

module servo_handle() {
	difference() {
		difference() {
			union() {
				cube([servo_handle_w, servo_handle_len-servo_handle_h, servo_handle_h]);
				for(s=[0, 1]) {
					translate([0, s*(servo_handle_len-servo_handle_h), servo_handle_h/2]) {
						rotate([0, 90, 0]) {
							cylinder(d=servo_handle_h, h=servo_handle_w);
						}
					}
				}
			}
			translate([pole_gap+wall, -servo_handle_h, 0]) {
				cube([servo_pole_w+2*pole_gap, servo_handle_len/2+servo_handle_h, servo_handle_h]);
			}
		}
		translate([-10, 0, servo_handle_h/2]) {
			rotate([0, 90, 0]) {
				cylinder(d=bolt_d_through, h=30);
			}
		}				cylinder(d=hole_d_through_m3, h=50, center=true);
				translate([-4, 0, 0]) {
					cylinder(d=hole_d_through_m3, h=50, center=true);
				}

		// servo hole
		translate([0, servo_handle_len-servo_handle_h, servo_handle_h/2]) {
			rotate([0, 90, 0]) {
				cylinder(d=servo_hole_d, h=servo_hole_sz);
				cylinder(d=servo_bolt_d, h=20);
			}
		}
	}
}

if(show_part_1) {
	pipette_pusher();
}
if(show_part_2) {
	translate([-servo_pole_w/2, 0, -servo_pole_h]) {
		servo_pole();
	}
}
if(show_part_3) {
	translate([-servo_handle_w/2, 0, -servo_pole_h+servo_handle_h/2+servo_pole_hole_off]) {
		rotate([180, 0, 0]) {
			translate([0, 0, 0]) {
				servo_handle();
			}
		}
	}
}	
