include <cnc_params.scad>
use <duratech_mockup.scad>
use <gantry_z.scad>
use <MCAD/nuts_and_bolts.scad>

$fn = 50;

wall = 3;
holder_sz_x = gantry_sz_x-rod_bearing_d-attach_sz_x-2*attach_side_gap;
holder_sz_y = 50;
holder_sz_z = 5;

holder_top_sz_z = 10;

holder_off_z = -32;

drill_center_off_y = -16;
drill_gap = 0.5;

module duratech_holder() {
	translate([0, drill_center_off_y-attach_sz_y/2-gantry_sz_y/2, -attach_sz_z/2+holder_sz_z/2+holder_off_z]) {
		difference() {
//		union() {
			union() {
				translate([0, drill_center_off_y, bottom_sz_z/2+above_bottom_sz_z/2+widest_level_sz_z/2]) {
					for(x=[-1,1]) {
						translate([x*(holder_sz_x+wall+1)/2, -drill_center_off_y-5, attach_sz_z/4]) {
							difference() {
								at_h = holder_sz_z+bottom_sz_z+above_bottom_sz_z+widest_level_sz_z+attach_sz_z/2;
								cube([wall, holder_sz_y/2+10, at_h], center=true);
								translate([0, -(holder_sz_y/2+10)/2, at_h/2]) {
									cube([20, holder_sz_y/2+10, at_h], center=true);
								}
							}
						}
					}
					translate([0, 0, -0.5]) {
						cylinder(d=widest_level_d+2*wall, h=holder_sz_z+bottom_sz_z+above_bottom_sz_z+widest_level_sz_z-1, center=true);
						translate([0, 4, 0]) {
							cube([holder_sz_x+2*wall+1, 20, holder_sz_z+bottom_sz_z+above_bottom_sz_z+widest_level_sz_z-1], center=true);
						}
					}
				}
			}
			translate([0, -default_attach_extra_sz_y-(drill_center_off_y-attach_sz_y/2-gantry_sz_y/2), -(-attach_sz_z/2+holder_sz_z/2+holder_off_z)]) {
				attachment_holes(d=hole_d_through);
			}
			translate([0, drill_center_off_y, 0]) {
				cylinder(d=metal_thing_d+2, h=20, center=true);
				translate([0, 0, holder_sz_z/2+bottom_sz_z/2]) {
					cylinder(d=bottom_d+2*drill_gap, h=bottom_sz_z, center=true);
					translate([0, 0, bottom_sz_z/2+above_bottom_sz_z/2]) {
						cylinder(d1=bottom_d+2*drill_gap, d2=above_bottom_d+2*drill_gap, h=above_bottom_sz_z, center=true);
						rotate([0, 0, -90]) {
							translate([above_bottom_box_sz_x, 0, 0]) {
								cube([above_bottom_box_sz_x, above_bottom_box_sz_y+2, above_bottom_box_sz_z+50], center=true);
							}
						}
						translate([0, 0, above_bottom_sz_z/2+widest_level_sz_z/2]) {
							cylinder(d1=above_bottom_d+2*drill_gap, d2=widest_level_d+2*drill_gap, h=widest_level_sz_z, center=true);
						}
					}
				}
			}
		}
	} 
}

module duratech_holder_top() {
	translate([0, drill_center_off_y-attach_sz_y/2-gantry_sz_y/2, holder_top_sz_z/2]) {
		difference() {
//		union() {
			union() {
				translate([0, drill_center_off_y, 0]) {
					at_h = attach_sz_z/2;
					for(x=[-1,1]) {
						translate([x*(holder_sz_x+wall+1)/2, -drill_center_off_y-5, at_h/2-holder_top_sz_z/2]) {
							difference() {
								cube([wall, holder_sz_y/2+10, at_h], center=true);
								translate([0, -(holder_sz_y/2+10)/2, at_h/2]) {
									cube([20, holder_sz_y/2+10, at_h], center=true);
								}
							}
						}
					}
					translate([0, 0, 0]) {
						cylinder(d=widest_level_d+2*wall, h=holder_top_sz_z, center=true);
						translate([0, 4, 0]) {
							cube([holder_sz_x+2*wall+1, 20, holder_top_sz_z], center=true);
						}
					}
				}
				translate([0, -widest_level_d/2-20, 0]) {
					cube([16, 20, holder_top_sz_z], center=true);
				}
			}
			translate([0, -default_attach_extra_sz_y-(drill_center_off_y-attach_sz_y/2-gantry_sz_y/2), -holder_top_sz_z/2]) {
				attachment_holes(d=hole_d_through);
			}
			translate([0, drill_center_off_y, 0]) {
				cylinder(d=widest_level_d+2*drill_gap, h=holder_top_sz_z, center=true);
			}
			translate([0, -widest_level_d/2-20, 0]) {
				cube([6, 20, holder_top_sz_z], center=true);
				translate([0, -5, 0]) {
					rotate([0, 90, 0]) {
						cylinder(d=hole_d_through, h=40, center=true);
						translate([0, 0, -16/2]) {
							nutHole(4);
						}
					}
				}
			}
		}
	} 
}

//gantry_z();
duratech_holder_top();
duratech_holder();
//color("blue") {
//	gantry_z_attachment();
//}
translate([0, -58, -36+holder_off_z]) {
	rotate([0, 0, -90]) {
		%duratech_mockup();
	}
}
