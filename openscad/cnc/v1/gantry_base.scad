$fn = 40;

rod_d = 12;
rod_bearing_d = 16; //?
rod_bearing_sz = 30; //?
rod_bearing_pos_z = [20, 50];
lead_screw_d = 8;
lead_screw_nut_d = 10; //?
lead_screw_nut_pos_z = 35; //?
lead_screw_nut_hole_d = 2.4; //?
lead_screw_nut_hole_r_dist = 4;

base_sz_x = rod_bearing_sz * 2;
base_sz_y = rod_bearing_d + 6; // shortest
base_sz_z = rod_bearing_pos_z[1] + rod_bearing_d;
side_sz_x = 4;
side_sz_z = base_sz_z + 16;
side_hole_d = 4.5;

module gantry_base() {

	difference() {
		union() {
			cube([base_sz_x, base_sz_y, base_sz_z], center=true);
			// sides
			for(xpos=[-1,1]) {
				translate([xpos*((base_sz_x-side_sz_x)/2), 0, 0]) {
//					difference() {
					union() {
						cube([side_sz_x, base_sz_y, side_sz_z], center=true);
						for(ypos=[-1,1]) {
							for(zpos=[-1,1]) {
								translate([0, ypos*(base_sz_y/2-side_hole_d-2), zpos*(side_sz_z/2-(side_sz_z-base_sz_z)/4)]) {
									translate([xpos*(side_sz_x+3)/2, 0, zpos*4]) {
										cube([side_sz_x+3, base_sz_y, side_hole_d/2], center=true);
									}
									rotate([0, 90, 0]) {
										cylinder(d=side_hole_d, h=side_sz_x*2, center=true);
									}
								}
							}
						}
					}
				}
			}
		}

		for(zpos=rod_bearing_pos_z) {
			translate([0, 0, -base_sz_z/2+zpos]) {
				rotate([0, 90, 0]) {
					cylinder(d=rod_bearing_d, h=base_sz_x, center=true);
				}
			}
		}

		translate([0, 0, -base_sz_z/2+lead_screw_nut_pos_z]) {
			rotate([0, 90, 0]) {
				cylinder(d=lead_screw_nut_d, h=base_sz_x, center=true);
				for(angle=[45:90:345]) {
//	echo(angle);
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

