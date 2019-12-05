include <cnc_params.scad>
use <gantry_z.scad>

$fn = 80;

wall = 5;

dc_motor_bottom_d = 45.5;
dc_motor_bearing_d = 17.5;
dc_motor_shaft_d = 5;
dc_motor_body_sz_z = 66.4;
dc_motor_bearing_sz_z = 4; //?
dc_motor_shaft_sz_z = 15; //?
dc_motor_bottom_holes_dist = 29;
dc_motor_bottom_hole_d = 4.6;

module dc_motor_mockup() {
	cylinder(d=dc_motor_bottom_d, h=dc_motor_body_sz_z);
	translate([0, 0, -dc_motor_bearing_sz_z]) {
		cylinder(d=dc_motor_bearing_d, h=dc_motor_bearing_sz_z);
		translate([0, 0, -dc_motor_shaft_sz_z]) {
			cylinder(d=dc_motor_shaft_d, h=dc_motor_shaft_sz_z);
		}
	}
}

module dc_motor_holder() {
	translate([0, -73, -23]) {
//		union() {
		difference() {
			translate([0, 0, -4]) {
				cylinder(d=dc_motor_bottom_d+2*wall, h=20);
				translate([0, (dc_motor_bottom_d/2+6)/2, 10]) {
					cube([dc_motor_bottom_d, dc_motor_bottom_d/2+6, 20], center=true);
				}
			}
			cylinder(d=dc_motor_bottom_d, h=20);
			translate([0, 0, -10]) {
				cylinder(d=dc_motor_bearing_d, h=20);
			}
			for(x=[-1,1]) {
				translate([x*dc_motor_bottom_holes_dist/2, 0, -4]) {
					cylinder(d=dc_motor_bottom_hole_d, h=20);
					cylinder(d2=dc_motor_bottom_hole_d, d1=8, h=3);
				}
			}
		}
		difference() {
//		union() {
			union() {
				at_h = 54;
				translate([0, 29, -4]) {
					for(x=[-1,1]) {
						translate([x*(holder_sz_x+2)/2, 0, 0]) {
							hull() {
								translate([0, -7, 10]) {
									cube([wall, 20, 20], center=true);
								}
								translate([0, 8, at_h/2]) {
									cube([wall, 16, at_h], center=true);
								}
							}
						}
					}
				}
			}
			translate([0, 58, 23]) {
				attachment_holes(d=hole_d_through);
			}
		}
	}
}

color("blue") {                                                                                                                                                                            
//	gantry_z_attachment();                                                                                                                                                                   
}                                                                                                                                                                                          
translate([0, 0, 0]) {
	dc_motor_holder();
}
//%dc_motor_mockup();
