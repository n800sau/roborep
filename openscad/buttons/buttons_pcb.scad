$fn=30;

hole_d = 2.8;
hole_d_through = 3.5;
wall = 2;
gap = 0.3;
pcb_hole_dist = 2.5;
pcb_double_sz_z = 1.5;
btn_sz_x = pcb_hole_dist * 3;
btn_sz_y = pcb_hole_dist * 2;
btn_sz_z = 5.2;
btn_pressed_sz_z = 4;
inter_btn_space = pcb_hole_dist;
btn_space_z = btn_sz_z + 2*wall;
presser_above_wall = 2;
button_link_sz_z = 0.5;

module button_presser() {
		cube([5, 5, gap+2*wall+presser_above_wall], center=true);
		translate([0, 0, (gap+wall+presser_above_wall)/2]) {
			cube([6, 6, gap+wall], center=true);
		}
}

module button_presser_hole() {
	cube([5+2*gap, 5+2*gap, wall], center=true);
}

// map of button center coordinates in pcb holes from left top corner
// zero based
module button_set(btn_coordinates=[], button_presser_mode=false, btn_links=[]) {
	translate([pcb_hole_off_x-pcb_sz_x/2, pcb_hole_off_y-pcb_sz_y/2, 0]) {
		for(i=[0:len(btn_coordinates)-1]) {
			p = btn_coordinates[i];
			translate([pcb_hole_dist*p[0], pcb_hole_dist*p[1], 0]) {
				if(button_presser_mode) {
					button_presser();
				} else {
					button_presser_hole();
				}
			}
		}
		if(button_presser_mode) {
			for(lnk=btn_links) {
				translate([0, 0, (2*gap+2*wall+presser_above_wall-button_link_sz_z)/2]) {
					hull() {
						p1 = btn_coordinates[lnk[0]];
						p2 = btn_coordinates[lnk[1]];
						translate([pcb_hole_dist*p1[0], pcb_hole_dist*p1[1], 0]) {
							cube([1, 1, button_link_sz_z], center=true);
							translate([pcb_hole_dist*(p2[0]-p1[0]), pcb_hole_dist*(p2[1]-p1[1]), 0]) {
								cube([1, 1, button_link_sz_z], center=true);
							}
						}
					}
				}
			}
		}
	}
}

button_set1 = [
				[1.5, 1],          [11.5, 1],
				[1.5, 4], [7.5, 4], [11.5, 4], [15.5, 4],
				[1.5, 7],          [11.5, 7],
					   [5.5, 8],
];

btn_links1 = [
	[0, 1], [1, 5],
	[2, 3], [4, 5],
	[6, 8], [8, 7],
  [3, 8], [0, 3],
];

pcb_sz_x = 50;
pcb_sz_y = 36.2;
pcb_hole_off_x = 3.5;
pcb_hole_off_y = 6;
pcb_side_free_space = 3;
side_gap = 5;
button_top_sz_x = pcb_sz_x+2*side_gap+2*gap+wall;
button_top_sz_y = pcb_sz_y+2*side_gap+2*gap+wall;
button_top_sz_z = wall+btn_space_z + pcb_double_sz_z;

module buttons_case() {
	difference() {
		union() {
			cube([button_top_sz_x, button_top_sz_y, wall], center=true);
			for(xsgn=[-1, 1]) {
				translate([xsgn*(button_top_sz_x-wall)/2, 0, (button_top_sz_z-wall)/2]) {
					cube([wall, button_top_sz_y, button_top_sz_z], center=true);
				}
			}
			for(ysgn=[-1, 1]) {
				translate([0, ysgn*(button_top_sz_y-wall)/2, (button_top_sz_z-wall)/2]) {
					cube([button_top_sz_x, wall, button_top_sz_z], center=true);
					// corners
					for(xsgn=[-1,1]) {
						translate([xsgn*(button_top_sz_x-wall-side_gap)/2, ysgn*-side_gap/2, 0]) {
							translate([0, ysgn*-wall, 0]) {
								cube([side_gap, side_gap+wall, button_top_sz_z], center=true);
							}
							translate([xsgn*-wall, 0, 0]) {
								cube([side_gap+wall, side_gap, button_top_sz_z], center=true);
							}
							hull() {
								cube([side_gap, side_gap, btn_space_z], center=true);
								translate([xsgn*(-side_gap-pcb_side_free_space)/2, ysgn*(-side_gap-pcb_side_free_space)/2, 0]) {
									cube([pcb_side_free_space, pcb_side_free_space, btn_space_z], center=true);
								}
							}
						}
					}
				}
			}
		}
		for(ysgn=[-1, 1]) {
			for(xsgn=[-1,1]) {
				translate([xsgn*(button_top_sz_x-wall-side_gap)/2, ysgn*(button_top_sz_y-wall-side_gap)/2, (button_top_sz_z-wall)/2]) {
					cylinder(d=hole_d_through, h=20, center=true);
				}
			}
		}
		button_set(button_set1);
	}
}

module pcb_cover() {
	difference() {
		cube([button_top_sz_x, button_top_sz_y, wall], center=true);
		cube([button_top_sz_x-2*wall-2*side_gap-2*pcb_side_free_space, button_top_sz_y-2*wall-2*side_gap-2*pcb_side_free_space, wall], center=true);
		for(ysgn=[-1, 1]) {
			for(xsgn=[-1,1]) {
				translate([xsgn*(button_top_sz_x-wall-side_gap)/2, ysgn*(button_top_sz_y-wall-side_gap)/2, (button_top_sz_z-wall)/2]) {
					cylinder(d=hole_d_through, h=20, center=true);
				}
			}
		}
	}
}

undercase_sz_z = 18;
attachment_width = 10;
attachment_external_length = 5;

module undercase() {
	difference() {
		union() {
			cube([button_top_sz_x, button_top_sz_y, wall], center=true);
			for(xsgn=[-1, 1]) {
				// wall along y
				translate([xsgn*(button_top_sz_x-wall)/2, 0, (undercase_sz_z-wall)/2]) {
					cube([wall, button_top_sz_y, undercase_sz_z], center=true);
				}
			}
			// attachment
			difference() {
				union() {
					cube([button_top_sz_x+2*wall+attachment_external_length, attachment_width, wall], center=true);
					for(xsgn=[-1, 1]) {
						translate([xsgn*(button_top_sz_x/2+wall+attachment_external_length/2), 0, 0]) {
							cylinder(d=attachment_width, h=wall, center=true);
						}
					}
				}
				for(xsgn=[-1, 1]) {
					translate([xsgn*(button_top_sz_x/2+wall+attachment_external_length/2), 0, 0]) {
						cylinder(d=hole_d_through, h=wall, center=true);
					}
				}
			}
			for(ysgn=[-1, 1]) {
				translate([0, ysgn*(button_top_sz_y-wall)/2, (undercase_sz_z-wall)/2]) {
					// wall along x
					cube([button_top_sz_x, wall, undercase_sz_z], center=true);
					// corners
					for(xsgn=[-1,1]) {
						translate([xsgn*(button_top_sz_x-wall-side_gap)/2, ysgn*-side_gap/2, 0]) {
							translate([0, 0, 0]) {
								cube([side_gap, side_gap, undercase_sz_z], center=true);
							}
						}
					}
				}
			}
		}
		for(ysgn=[-1, 1]) {
			for(xsgn=[-1,1]) {
				translate([xsgn*(button_top_sz_x-wall-side_gap)/2, ysgn*(button_top_sz_y-wall-side_gap)/2, (undercase_sz_z-wall)/2]) {
					cylinder(d=hole_d, h=undercase_sz_z, center=true);
				}
			}
		}
		// hole for wires
		translate([button_top_sz_x/2, 0, undercase_sz_z-3.5]) {
			cube([10, 10, 7], center=true);
		}
	}
}

translate([0, 0, undercase_sz_z+button_top_sz_z-wall]) {
	rotate([180, 0, 0]) {
		//buttons_case();
	}
}
//undercase();

//button_presser();
button_set(button_set1, button_presser_mode=true, btn_links=btn_links1);
translate([0, 0, button_top_sz_z]) {
	//pcb_cover();
}
