include <cnc_params.scad>
use <MCAD/boxes.scad>

$fn = 30;

gantry_sz_x = base_sz_x;
gantry_sz_y = rod_bearing_d + 2 * wall;
gantry_sz_z = rod_bearing_sz * 2 + 4;

attach_sz_x = wall;
attach_sz_y = 20;
attach_sz_z = 10;

extract_sz_x = lead_screw_nut_hole_r_dist * 2 + 2 * wall;
extract_sz_y = extract_sz_x + wall;
extract_sz_z = gantry_sz_z / 2 - 10;

module vert_rod_holes(h=top_block_sz_z, z_off=0) {
	for(xpos=[-1,1]) {
		// rod holes
		translate([xpos*23.25, 0, z_off]) {
			cylinder(d=rod_d, h=h, center=true);
		}
	}
}

module gantry() {
	difference() {
		union() {
			cube([gantry_sz_x, gantry_sz_y, gantry_sz_z], center=true);
			// attachment
			for(xsgn=[-1,1]) {
				for(zsgn=[-1,1]) {
					translate([xsgn*(gantry_sz_x-attach_sz_x)/2, -gantry_sz_y/2, zsgn*(rod_bearing_sz-attach_sz_z/2)]) {
						difference() {
							union() {
								cube([attach_sz_x, attach_sz_y, attach_sz_z], center=true);
								translate([0, -attach_sz_y/2, 0]) {
									rotate([0, 90, 0]) {
										cylinder(d=attach_sz_z, h=attach_sz_x, center=true);
									}
								}
							}
							translate([0, -attach_sz_y/2, 0]) {
								rotate([0, 90, 0]) {
									cylinder(d=hole_d_through, h=attach_sz_x*2, center=true);
								}
							}
						}
					}
				}
			}
		}

		translate([0, motor_shift_y, 0]) {
			// motor hole (lead)
			cylinder(d=lead_screw_d, h=gantry_sz_z, center=true);
			// motor hole (lead attachment)
			for(angle=[45:90:345]) {
				rotate([0, 0, angle]) {
					translate([lead_screw_nut_hole_r_dist, 0, 0]) {
						cylinder(d=lead_screw_nut_hole_d, h=gantry_sz_z, center=true);
					}
				}
			}
			// extraction from top and bottom
			for(zpos=[-1,1]) {
				translate([0, wall/2, zpos*(gantry_sz_z-extract_sz_z+1)/2]) {
					roundedBox([extract_sz_x, extract_sz_y, extract_sz_z], 2, true);
				}
			}
		}

		// holes for rod bearings
		vert_rod_holes(h=gantry_sz_z);
	}
}

gantry();
