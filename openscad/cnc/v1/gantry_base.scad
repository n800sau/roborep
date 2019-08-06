include <cnc_params.scad>

module mounting_holes() {
	for(ypos=[-1, 1]) {
		translate([0, ypos * (hole_d+2), 0]) {
			rotate([0, 90, 0]) {
				cylinder(d=hole_d, h=base_sz_x*2, center=true);
			}
		}
	}
	rotate([90, 0, 0]) {
		cylinder(d=hole_d, h=base_sz_x*2, center=true);
	}
}

module gantry_base() {

	difference() {

		cube([base_sz_x, base_sz_y, base_sz_z], center=true);

		// holes for top and bottom mounting
		for(zpos=[-1, 1]) {
			translate([0, 0, zpos*(base_sz_z/2-5)]) {
				mounting_holes();
			}
		}

		// holes for rod bearings
		for(zpos=rod_bearing_pos_z) {
			translate([0, 0, -base_sz_z/2+zpos]) {
				rotate([0, 90, 0]) {
					cylinder(d=rod_bearing_d, h=base_sz_x, center=true);
				}
			}
		}

		// holes for the screw lead
		translate([0, 0, -base_sz_z/2+lead_screw_nut_pos_z]) {
			rotate([0, 90, 0]) {
				cylinder(d=lead_screw_nut_d, h=base_sz_x, center=true);
				for(angle=[45:90:345]) {
					rotate([0, 0, angle]) {
						translate([lead_screw_nut_hole_r_dist, 0, 0]) {
							cylinder(d=lead_screw_nut_hole_d, h=base_sz_x, center=true);
						}
					}
				}
			}
		}
	}

}

rotate([0, 0, 45]) {
	gantry_base();
}

