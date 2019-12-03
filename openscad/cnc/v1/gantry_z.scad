include <cnc_params.scad>
include <MCAD/nuts_and_bolts.scad>

$fn = 50;

module vert_rod_holes(d=rod_d, h=top_block_sz_z, z_off=0) {
	for(xpos=[-1,1]) {
		// rod holes
		translate([xpos*rod_dist_x/2, 0, z_off]) {
			cylinder(d=d, h=h, center=true);
		}
	}
}

module nutHoleM4(extra_h=0) {
	hull() {
		translate([0, 0, extra_h/2]) nutHole(4, units=MM);
		translate([0, 0, -extra_h/2]) nutHole(4, units=MM);
	}
}

module attachment_hole(d=hole_d, nut_hole_sz=0, size=gantry_sz_x) {
	rotate([0, 90, 0]) {
		h = size*2;
		cylinder(d=d, h=h, center=true);
		if(nut_hole_sz > 0) {
			translate([0, 0, -METRIC_NUT_THICKNESS[4]/2]) {
				nutHoleM4(extra_h=nut_hole_sz);
			}
		}
	}
}

module attachment_holes(d=hole_d, hole_count=4, nut_hole_sz=0) {
	z_step = 15;
	translate([0, -gantry_sz_y/2-attach_sz_y/4, -(gantry_sz_z-attach_sz_z)/2-attach_sz_z/2]) {
		for(zi=[0:1:hole_count-1]) {
			translate([0, 0, 10+zi*z_step]) {
				attachment_hole(d=d, nut_hole_sz=nut_hole_sz);
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
	// pointer to the work surface
	translate([0, 0, -(gantry_sz_z+gantry_z_to_work_surface_sz_z)/2]) {
		%cube([5, 5, gantry_z_to_work_surface_sz_z], center=true);
	}
}


module gantry_z_attachment(extra_sz_y=default_attach_extra_sz_y) {
	difference() {
		translate([0, -(gantry_sz_y+connector_base_sz_y)/2-attach_side_gap, 0]) {
			cube([connector_base_sz_x, connector_base_sz_y, attach_sz_z], center=true);
			translate([0, -(connector_base_sz_y+extra_sz_y)/2, 0]) {
				difference() {
					cube([connector_base_sz_x, extra_sz_y, attach_sz_z], center=true);
					cube([connector_base_sz_x-2*connector_connector_sz_x-2*nuts_sz_z, extra_sz_y, attach_sz_z], center=true);
				}
			}
		}
		attachment_holes(d=hole_d, hole_count=20);
		if(extra_sz_y >= 10) {
			translate([0, -extra_sz_y, 0]) {
				attachment_holes(d=hole_d_through, hole_count=20);
			}
		}
	}
}

nut_hole_sz = connector_base_sz_x-2*connector_connector_sz_x-nuts_sz_z;

module gantry_z_attachment_sides(extra_sz_y=default_attach_extra_sz_y, extra_sz_z=0) {
	attach_ext_sz_z = attach_sz_z + 6;
	difference() {
		for(xside=[-1,1]) {
			translate([xside*(connector_base_sz_x-connector_connector_sz_x-nuts_sz_z)/2, -(gantry_sz_y+connector_base_sz_y)/2-attach_side_gap, 0]) {
				cube([connector_connector_sz_x+nuts_sz_z, connector_base_sz_y, attach_ext_sz_z], center=true);
				translate([0, -(connector_base_sz_y+extra_sz_y)/2, extra_sz_z/2]) {
					cube([connector_connector_sz_x+nuts_sz_z, extra_sz_y, attach_ext_sz_z+extra_sz_z], center=true);
				}
			}
		}
		attachment_holes(d=hole_d_through, hole_count=20, nut_hole_sz=nut_hole_sz);
		if(extra_sz_y >= 10) {
			translate([0, -extra_sz_y, 0]) {
				attachment_holes(d=hole_d_through, hole_count=20, nut_hole_sz=nut_hole_sz);
			}
		}
	}
}

module sides_bracket(extra_sz_y=default_attach_extra_sz_y) {
	translate([0, -gantry_sz_y-10, 83]) {  
		difference() {
	//	union() {
			union() {
				for(x=[-1,1]) {
					translate([x*(connector_base_sz_x/2-(connector_connector_sz_x+nuts_sz_z)*1.5), wall, 0]) {
						cube([connector_connector_sz_x+nuts_sz_z, connector_base_sz_y+2, 12], center=true);
					}
				}
				translate([0, (connector_base_sz_y+wall+2)/2, 0]) {
					cube([connector_base_sz_x/2-(connector_connector_sz_x+nuts_sz_z), wall, 12], center=true);
				}
			}
			translate([0, 2, 0]) {
				attachment_hole(d=hole_d_through, nut_hole_sz=nut_hole_sz-3-(connector_connector_sz_x+nuts_sz_z));
			}
		}
	}
}

//gantry_z();
//color("blue") {
	//gantry_z_attachment(extra_sz_y=default_attach_extra_sz_y);
	gantry_z_attachment_sides(extra_sz_y=default_attach_extra_sz_y, extra_sz_z=attach_sz_z+10+40);
	translate([0, 20,0]) {
		sides_bracket();
	}
//}

