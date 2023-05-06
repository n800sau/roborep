show_slider = 0;
show_part1 = 0;
show_part2 = 1;
show_part3 = 1;

include <cnc_params.scad>
include <MCAD/nuts_and_bolts.scad>

$fn = 50;
insert_d = 5.8;

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

module attachment_hole(d=hole_d_m4, nut_hole_sz=0, size=slider_sz_x, insert_hole_mode=false) {
	rotate([0, 90, 0]) {
		h = size*2;
		cylinder(d=insert_hole_mode ? insert_d : d, h=h, center=true);
		if(nut_hole_sz > 0 && !insert_hole_mode) {
			translate([0, 0, -METRIC_NUT_THICKNESS[4]/2]) {
				nutHoleM4(extra_h=nut_hole_sz);
			}
		}
	}
}

module attachment_holes(d=hole_d_m4, hole_count=4, nut_hole_sz=0, insert_hole_mode=false) {
	z_step = 15;
	translate([0, -slider_sz_y/2-attach_sz_y/4, -(slider_sz_z-attach_sz_z)/2-attach_sz_z/2]) {
		for(zi=[0:1:hole_count-1]) {
			translate([0, 0, 10+zi*z_step]) {
				attachment_hole(d=d, nut_hole_sz=nut_hole_sz, insert_hole_mode=insert_hole_mode);
			}
		}
	}
}

module slider_z() {
	difference() {
		union() {
			cube([slider_sz_x-rod_bearing_d+2*wall, slider_sz_y, slider_sz_z], center=true);
			translate([0, wall, 0]) {
				cube([lead_screw_nut_attach_plate_d, slider_sz_y, slider_sz_z], center=true);
			}
			// attachment
			difference() {
				translate([0, -slider_sz_y/2, -(slider_sz_z-attach_sz_z)/2]) {
					for(xsgn=[-1,1]) {
						translate([xsgn*(slider_sz_x-rod_bearing_d)/2, 0, 0]) {
							union() {
								cube([attach_sz_x, attach_sz_y, attach_sz_z], center=true);
								translate([0, (-slider_sz_z+attach_sz_z)/4, attach_sz_z/2]) {
									rotate([0, 90, 0]) {
										cylinder(r=slider_sz_z-attach_sz_z, h=attach_sz_x, center=true);
									}
								}
							}
						}
					}
				}
				attachment_holes(d=hole_d_through_m4);
			}
			// holes for rod bearings
			vert_rod_holes(d=rod_bearing_d+2*wall, h=slider_sz_z);
		}

		translate([0, motor_shift_y, 0]) {
			// motor hole (lead)
			cylinder(d=lead_screw_nut_d, h=slider_sz_z, center=true);
			// motor hole (lead attachment)
			for(angle=[45:90:345]) {
				rotate([0, 0, angle]) {
					translate([lead_screw_nut_hole_r_dist, 0, 0]) {
						cylinder(d=lead_screw_nut_hole_d, h=slider_sz_z, center=true);
					}
				}
			}
			// extraction from top
			for(zpos=[1]) {
				translate([0, 0, zpos*(slider_sz_z-extract_sz_z+1)/2]) {
					cylinder(d=lead_screw_nut_attach_plate_d+2, h=extract_sz_z, center=true);
					translate([0, lead_screw_nut_attach_plate_d/2, 0]) {
						cube([lead_screw_nut_attach_plate_d+2, lead_screw_nut_attach_plate_d+wall, extract_sz_z], center=true);
					}
				}
			}
		}

		// holes for rod bearings
		vert_rod_holes(d=rod_bearing_d, h=slider_sz_z);
	}
	// pointer to the work surface
	translate([0, 0, -(slider_sz_z+slider_z_to_work_surface_sz_z)/2]) {
//		%cube([5, 5, slider_z_to_work_surface_sz_z], center=true);
	}
}


module slider_z_attachment(extra_sz_y=default_attach_extra_sz_y) {
	difference() {
		translate([0, -(slider_sz_y+connector_base_sz_y)/2-attach_side_gap, 0]) {
			cube([connector_base_sz_x, connector_base_sz_y, attach_sz_z], center=true);
			translate([0, -(connector_base_sz_y+extra_sz_y)/2, 0]) {
				difference() {
					cube([connector_base_sz_x, extra_sz_y, attach_sz_z], center=true);
					cube([connector_base_sz_x-2*connector_connector_sz_x-2*nuts_sz_z, extra_sz_y, attach_sz_z], center=true);
				}
			}
		}
		attachment_holes(d=hole_d_m4, hole_count=20);
		if(extra_sz_y >= 10) {
			translate([0, -extra_sz_y, 0]) {
				attachment_holes(d=hole_d_through_m4, hole_count=20);
			}
		}
	}
}

nut_hole_sz = connector_base_sz_x-2*connector_connector_sz_x-nuts_sz_z;

module slider_z_adapter(side="all", extra_sz_y=default_attach_extra_sz_y, extra_sz_z=0, insert_hole_mode=false) {
	attach_ext_sz_z = attach_sz_z + 6;
	difference() {
		for(xside=[-1,1]) {
			if(
					((side == "all" || side == "left") && xside==1) ||
					((side == "all" || side == "right") && xside==-1)
			) {
				translate([xside*(connector_base_sz_x-connector_connector_sz_x-nuts_sz_z)/2, -(slider_sz_y+connector_base_sz_y)/2-attach_side_gap, 0]) {
					cube([connector_connector_sz_x+nuts_sz_z, connector_base_sz_y, attach_ext_sz_z], center=true);
					translate([0, -(connector_base_sz_y+extra_sz_y)/2, extra_sz_z/2]) {
						cube([connector_connector_sz_x+nuts_sz_z, extra_sz_y, attach_ext_sz_z+extra_sz_z], center=true);
					}
				}
			}
		}
		attachment_holes(d=hole_d_through_m4, hole_count=20, nut_hole_sz=nut_hole_sz, insert_hole_mode=insert_hole_mode);
		if(extra_sz_y >= 10) {
			translate([0, -extra_sz_y, 0]) {
				attachment_holes(d=hole_d_through_m4, hole_count=20, nut_hole_sz=nut_hole_sz, insert_hole_mode=insert_hole_mode);
			}
		}
	}
}

module adapter_bracket(extra_sz_y=default_attach_extra_sz_y) {
	translate([0, -slider_sz_y-10, 83]) {  
		difference() {
	//	union() {
			union() {
				for(x=[-1,1]) {
					translate([x*(connector_base_sz_x/2-connector_connector_sz_x-connector_connector_sz_x/2-nuts_sz_z), wall, 0]) {
						cube([connector_connector_sz_x, connector_base_sz_y+2, 12], center=true);
					}
				}
				translate([0, (connector_base_sz_y+wall+2)/2, 0]) {
					cube([connector_base_sz_x/2-connector_connector_sz_x+1, wall, 12], center=true);
				}
			}
			translate([0, 2, 0]) {
				attachment_hole(d=hole_d_through_m4, nut_hole_sz=nut_hole_sz-6-(connector_connector_sz_x+nuts_sz_z));
			}
		}
	}
}

if(show_slider) {
	slider_z();
}
if(show_part1) {
	slider_z_attachment(extra_sz_y=default_attach_extra_sz_y);
}
if(show_part2) {
	slider_z_adapter(side="left", extra_sz_y=default_attach_extra_sz_y, extra_sz_z=attach_sz_z+10+26, insert_hole_mode=true);
}
if(show_part3) {
	slider_z_adapter(side="right", extra_sz_y=default_attach_extra_sz_y, extra_sz_z=attach_sz_z+10+26, insert_hole_mode=true);
}
// does not fit
translate([0, 10,0]) {
	//	adapter_bracket();
}

