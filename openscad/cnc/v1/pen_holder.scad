include <cnc_params.scad>

$fn = 50;

holder_sz_y = 15;

module pen_holder() {
			// attachment
			for(xsgn=[-1,1]) {
				translate([xsgn*(gantry_sz_x-rod_bearing_d+attach_sz_x)/2, 0, 0]) {
					difference() {
						union() {
							cube([attach_sz_x, holder_sz_y, attach_sz_z], center=true);
						}
						translate([0, 0, -attach_sz_z/2]) {
							for(zpos=[10, 25, 40]) {
								translate([0, 0, zpos]) {
									rotate([0, 90, 0]) {
										cylinder(d=hole_d_through, h=attach_sz_x*2, center=true);
									}
								}
							}
						}
					}
				}
			}
							translate([0, 0, (-attach_sz_z+wall)/2]) {
								difference() {
									union() {
										hull() {
											cube([gantry_sz_x-rod_bearing_d+attach_sz_x, holder_sz_y, wall], center=true);
											translate([0, holder_sz_y, (10-wall)/2]) {
												cylinder(d=10, h=20, center=true);
											}
										}
										translate([0, holder_sz_y, (30-wall)/2]) {
											cylinder(d=10, h=30, center=true);
										}
									}
									translate([0, holder_sz_y, 15]) {
										translate([0, 2.5, 0]) {
											cube([20, 5, 30], center=true);
										}
										cylinder(d=5, h=30, center=true);
									}
								}
							}
}

pen_holder();

