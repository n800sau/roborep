include <cnc_params.scad>
use <gantry_x.scad>
use <gantry_z.scad>
use <MCAD/boxes.scad>
use <MCAD/triangles.scad>

extra_border_sz_z = 15;
motor_raise_sz_z = 16;
motor_raise_wall_sz = 4;

triangle_sz_y = 20;
top_block_reduce_y = 15;
motor_bolt_hole_d = 4;

module motor_bolt_holes(d=motor_bolt_hole_d) {
	for(xpos=[-1,1]) {
		for(ypos=[-1,1]) {
			// motor bolt holes
			translate([xpos*motor_hole_dist/2, ypos*motor_hole_dist/2, 0]) {
				cylinder(d=d, h=top_block_sz_z*4, center=true);
			}
		}
	}
}

module	motor_raise() {
	translate([0, -(top_block_sz_y-motor_sz)/2+wall+motor_shift_y, (motor_raise_sz_z+top_block_sz_z)/2]) {
		difference() {
			translate([0, 0, (wall-motor_raise_sz_z)/2]) {
				roundedBox([motor_sz, motor_sz, wall], 2, true);
				translate([0, 0, (motor_raise_sz_z-wall)/2]) {
					rotate([0, 0, 45]) {
						difference() {
							roundedBox([motor_sz-3*motor_raise_wall_sz, motor_sz-3*motor_raise_wall_sz, motor_raise_sz_z], 2, true);
							roundedBox([motor_sz-4*motor_raise_wall_sz, motor_sz-4*motor_raise_wall_sz, motor_raise_sz_z], 2, true);
						}
					}
				}
			}
			motor_bolt_holes(d=motor_bolt_hole_d);
			motor_center_hole(d=motor_d);
		}
	}
}

module motor_center_hole(d=motor_d) {
	cylinder(d=d, h=top_block_sz_z*4, center=true);
}

module motor_side() {
	difference() {
		union() {
			cube([top_block_sz_x, top_block_sz_y - top_block_reduce_y, top_block_sz_z], center=true);
			translate([0, 0, -(extra_border_sz_z)/2]) {
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
				// side support triangle
				for(x=[-1,1]) {
					translate([x*(top_block_sz_x-wall)/2+wall/2, (triangle_sz_y-wall)/2, wall]) {
						rotate([0, 90, 180]) {
							triangle(triangle_sz_y-5, extra_border_sz_z, wall);
						}
					}
				}
			}
		}
		// corner cuts
		for(x=[-1,1]) {
			translate([x*(top_block_sz_x-25), -top_block_sz_y/2, 0]) {
				rotate([0, 0, 45]) {
					cube([50, 50, top_block_sz_z], center=true);
				}
			}
		}
		translate([0, (top_block_sz_y-base_sz_y)/2-wall, 0]) {
			cube([base_sz_x+wall, base_sz_y+1, top_block_sz_z+extra_border_sz_z*2], center=true);
			translate([0, 0, top_block_sz_z/2-mounting_hole_offset]) {
				mounting_holes(d=hole_d_through, cone=true);
			}
			translate([0, 0, -1]) {
				holes_just_in_case(d=hole_d_through, cone=true);
			}
		}
		translate([0, -(top_block_sz_y-motor_sz)/2+wall, top_block_sz_z/4]) {
			translate([0, motor_shift_y, 0]) {
				motor_center_hole();
				motor_bolt_holes();
			}
			vert_rod_holes(z_off=-top_block_sz_z/4-wall);
		}
	}
}

motor_raise();
motor_side();
