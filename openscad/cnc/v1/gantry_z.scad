include <cnc_params.scad>

$fn = 50;

module vert_rod_holes(d=rod_d, h=top_block_sz_z, z_off=0) {
	for(xpos=[-1,1]) {
		// rod holes
		translate([xpos*rod_dist_x/2, 0, z_off]) {
			cylinder(d=d, h=h, center=true);
		}
	}
}

module attachment_holes(d=hole_d) {
	translate([0, -gantry_sz_y/2-attach_sz_y/4, -(gantry_sz_z-attach_sz_z)/2-attach_sz_z/2]) {
		for(zpos=[10, 25, 40, 55]) {
			translate([0, 0, zpos]) {
				rotate([0, 90, 0]) {
					cylinder(d=hole_d_through, h=gantry_sz_x*2, center=true);
				}
			}
		}
	}
}

module gantry_z() {
	difference() {
		union() {
			cube([gantry_sz_x-rod_bearing_d+2*wall, gantry_sz_y, gantry_sz_z], center=true);
			translate([0, wall, 0]) {
				cube([lead_screw_nut_attach_plate_d, gantry_sz_y, gantry_sz_z], center=true);
			}
			// attachment
			difference() {
				translate([0, -gantry_sz_y/2, -(gantry_sz_z-attach_sz_z)/2]) {
					for(xsgn=[-1,1]) {
						translate([xsgn*(gantry_sz_x-rod_bearing_d)/2, 0, 0]) {
							union() {
								cube([attach_sz_x, attach_sz_y, attach_sz_z], center=true);
								translate([0, (-gantry_sz_z+attach_sz_z)/4, attach_sz_z/2]) {
									rotate([0, 90, 0]) {
										cylinder(r=gantry_sz_z-attach_sz_z, h=attach_sz_x, center=true);
									}
								}
							}
						}
					}
				}
				attachment_holes(d=hole_d_through);
			}
			// holes for rod bearings
			vert_rod_holes(d=rod_bearing_d+2*wall, h=gantry_sz_z);
		}

		translate([0, motor_shift_y, 0]) {
			// motor hole (lead)
			cylinder(d=lead_screw_nut_d, h=gantry_sz_z, center=true);
			// motor hole (lead attachment)
			for(angle=[45:90:345]) {
				rotate([0, 0, angle]) {
					translate([lead_screw_nut_hole_r_dist, 0, 0]) {
						cylinder(d=lead_screw_nut_hole_d, h=gantry_sz_z, center=true);
					}
				}
			}
			// extraction from top
			for(zpos=[1]) {
				translate([0, 0, zpos*(gantry_sz_z-extract_sz_z+1)/2]) {
					cylinder(d=lead_screw_nut_attach_plate_d+2, h=extract_sz_z, center=true);
					translate([0, lead_screw_nut_attach_plate_d/2, 0]) {
						cube([lead_screw_nut_attach_plate_d+2, lead_screw_nut_attach_plate_d+wall, extract_sz_z], center=true);
					}
				}
			}
		}

		// holes for rod bearings
		vert_rod_holes(d=rod_bearing_d, h=gantry_sz_z);
	}
}

module gantry_z_attachment() {
	union() {
		translate([0, -(gantry_sz_y+attach_sz_y)/2, 0]) {
			cube([gantry_sz_x-rod_bearing_d-attach_sz_x-1, attach_sz_y, attach_sz_z], center=true);
		}
		attachment_holes(d=hole_d);
	}
}


gantry_z();
color("blue")
	gantry_z_attachment();
