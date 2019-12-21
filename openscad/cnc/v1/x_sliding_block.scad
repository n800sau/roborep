include <cnc_params.scad>

module mounting_holes(d=hole_d, cone=false) {
	for(ypos=[-1, 1]) {
		translate([0, ypos * (hole_d+2), 0]) {
			rotate([0, 90, 0]) {
				cylinder(d=d, h=base_sz_x+2*wall+10, center=true);
				if(cone) {
					translate([0, 0, base_sz_x/2+wall]) {
						cylinder(d1=d, d2=d+4, h=3, center=true);
					}
					translate([0, 0, -base_sz_x/2-wall]) {
						cylinder(d2=d, d1=d+4, h=3, center=true);
					}
				}
			}
		}
	}
	for(xpos=[-1, 0, 1]) {
		translate([xpos*15, 0, 0]) {
			rotate([90, 0, 0]) {
				cylinder(d=d, h=base_sz_y+2*wall, center=true);
		if(cone) {
			translate([0, 0, -base_sz_y/2-wall]) {
				cylinder(d2=d, d1=d+4, h=3, center=true);
			}
			translate([0, 0, base_sz_y/2+wall]) {
				cylinder(d1=d, d2=d+4, h=3, center=true);
			}
		}
			}
		}
	}
}

module holes_just_in_case(d=hole_d, cone=false) {
	for(zoff=[20, 30, 40, 50, 93, 119]) {
		translate([0, 0, base_sz_z/2-zoff]) {
			mounting_holes(d=d, cone=cone);
		}
	}
}

module slider_x() {

	difference() {

		cube([base_sz_x, base_sz_y, base_sz_z], center=true);

		// holes for top and bottom mounting
		for(zpos=[-1, 1]) {
			translate([0, 0, zpos*(base_sz_z/2-mounting_hole_offset)]) {
				mounting_holes();
			}
		}

		holes_just_in_case();

		// holes for rod bearings
		for(zpos=rod_bearing_pos_z) {
			translate([0, 0, -base_sz_z/2+zpos]) {
				rotate([0, 90, 0]) {
					cylinder(d=rod_bearing_d, h=base_sz_x+wall, center=true);
				}
			}
		}

		// holes for the screw lead
		translate([0, 0, -base_sz_z/2+lead_screw_nut_pos_z]) {
			rotate([0, 90, 0]) {
				cylinder(d=lead_screw_nut_d, h=base_sz_x+wall, center=true);
				for(angle=[45:90:345]) {
					rotate([0, 0, angle]) {
						translate([lead_screw_nut_hole_r_dist, 0, 0]) {
							cylinder(d=lead_screw_nut_hole_d, h=base_sz_x+wall, center=true);
						}
					}
				}
			}
		}
	}

}

rotate([0, 0, 90]) {
	slider_x();
}

//translate([4.5, -200, -140]) {
//	import("spindle_carriage.stl", convexity = 5);
//}
