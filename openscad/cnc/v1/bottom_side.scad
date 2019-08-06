include <cnc_params.scad>
use <gantry_base.scad>

module bottom_side() {
	difference() {
		cube([bottom_block_sz_x, bottom_block_sz_y, bottom_block_sz_z], center=true);
		translate([0, (bottom_block_sz_y-base_sz_y)/2-wall, 0]) {
			cube([base_sz_x, base_sz_y, bottom_block_sz_z], center=true);
			mounting_holes();
		}
		translate([0, -(bottom_block_sz_y-motor_sz)/2+wall, bottom_block_sz_z/4]) {
			// motor hole
			translate([0, 0, bottom_block_sz_z/2]) {
				cylinder(d=lead_screw_bearing_d, h=bottom_block_sz_z, center=true);
			}
			cylinder(d=lead_screw_d, h=bottom_block_sz_z*4, center=true);
			for(xpos=[-1,1]) {
				// rod holes
				translate([xpos*(motor_sz/2+(bottom_block_sz_x-motor_sz)/4), 0, bottom_block_sz_z/2]) {
					cylinder(d=rod_bearing_d, h=bottom_block_sz_z, center=true);
				}
			}
		}
	}
}

bottom_side();
