include <cnc_params.scad>
use <gantry_base.scad>

hole_sz_x = base_sz_x-1;
hole_sz_y = base_sz_y+0.8;
echo("hole sz x: ", hole_sz_x, ", hole sz y:", hole_sz_y);

module bottom_side() {
	difference() {
		cube([bottom_block_sz_x, bottom_block_sz_y, bottom_block_sz_z], center=true);
		translate([0, (bottom_block_sz_y-base_sz_y)/2-wall, 0]) {
			cube([hole_sz_x, hole_sz_y, bottom_block_sz_z], center=true);
			translate([0, 0, (-bottom_block_sz_z/2)+mounting_hole_offset]) {
				mounting_holes(d=hole_d_through, cone=true);
			}
		}
		translate([0, -(top_block_sz_y-bottom_block_sz_y)/2-(top_block_sz_y-motor_sz)/2+wall, bottom_block_sz_z/2]) {
			// motor hole (bearing)
			translate([0, 0, -lead_screw_bearing_sz/2]) {
				cylinder(d=lead_screw_bearing_d, h=lead_screw_bearing_sz, center=true);
			}
			// motor hole (lead)
			cylinder(d=lead_screw_d, h=bottom_block_sz_z*2, center=true);
			for(xpos=[-1,1]) {
				// rod holes
				translate([xpos*rod_distance/2, 0, 0]) {
					cylinder(d=rod_d, h=bottom_block_sz_z, center=true);
				}
			}
		}
	}
}

bottom_side();
