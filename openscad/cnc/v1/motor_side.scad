include <cnc_params.scad>
use <gantry_base.scad>

extra_border_sz_z = 20;

module motor_side() {
	difference() {
		union() {
			cube([top_block_sz_x, top_block_sz_y, top_block_sz_z], center=true);
			translate([0, 0, (top_block_sz_z+extra_border_sz_z)/2]) {
				for(y = [0, 1]) {
					translate([0, (top_block_sz_y-wall)/2-y*(base_sz_y+wall), 0]) {
						cube([top_block_sz_x, wall, top_block_sz_z + extra_border_sz_z], center=true);
					}
				}
				for(x = [-1, 1]) {
					translate([x*top_block_sz_x/2, (top_block_sz_y-wall-base_sz_y-wall)/2, 0]) {
						cube([wall, base_sz_y+2*wall, top_block_sz_z + extra_border_sz_z], center=true);
					}
				}
			}
		}
		translate([0, (top_block_sz_y-base_sz_y)/2-wall, 0]) {
			cube([base_sz_x, base_sz_y, top_block_sz_z], center=true);
			translate([0, 0, top_block_sz_z/2-mounting_hole_offset]) {
				mounting_holes(d=hole_d_through);
			}
		}
		translate([0, -(top_block_sz_y-motor_sz)/2+wall, top_block_sz_z/4]) {
//			cube([motor_sz, motor_sz, top_block_sz_z/2], center=true);
			cylinder(d=motor_d, h=top_block_sz_z*4, center=true);
			for(xpos=[-1,1]) {
				for(ypos=[-1,1]) {
					// motor bold holes
					translate([xpos*motor_hole_dist/2, ypos*motor_hole_dist/2, 0]) {
						cylinder(d=hole_d, h=top_block_sz_z*4, center=true);
					}
				}
				// rod holes
				translate([xpos*23.25, 0, -top_block_sz_z/4-wall]) {
					cylinder(d=rod_d, h=top_block_sz_z, center=true);
				}
			}
		}
	}
}

motor_side();
