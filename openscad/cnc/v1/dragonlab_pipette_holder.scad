include <cnc_params.scad>
use <dragonlab_pipette_mockup.scad>
use <gantry_z.scad>
use <MCAD/nuts_and_bolts.scad>

$fn = 50;

wall = 3;
holder_sz_x = gantry_sz_x-rod_bearing_d-attach_sz_x-2*attach_side_gap;
holder_sz_y = 50;
holder_sz_z = 5;

holder_top_sz_z = 10;

holder_off_z = 0;

pipette_center_off_y = -16;
pipette_gap = 0.5;

module pipette_bottom_holder() {
	translate([0, pipette_center_off_y-attach_sz_y/2-gantry_sz_y/2, -attach_sz_z/2+holder_sz_z/2+holder_off_z]) {
		difference() {
//		union() {
			union() {
				translate([0, pipette_center_off_y, bottom_sz_z/2+above_bottom_sz_z/2]) {
					for(x=[-1,1]) {
						translate([x*(holder_sz_x+wall+1)/2, -pipette_center_off_y-5, attach_sz_z-9]) {
							difference() {
								cube([wall, holder_sz_y/2+10, attach_sz_z/2], center=true);
								translate([0, -(holder_sz_y/2+10)/2, pipette_bottom_holder_sz_z]) {
									cube([20, holder_sz_y/2+10, attach_sz_z/2], center=true);
								}
							}
						}
						translate([0, 0, attach_sz_z-pipette_bottom_holder_sz_z]) {
							difference() {
								cube([holder_sz_x+2*wall+1, pipette_bottom_holder_sz_y+2*wall, pipette_bottom_holder_sz_z], center=true);
								hull() {
									translate([0, 0, wall]) {
										cube([pipette_bottom_holder_sz_x, pipette_bottom_holder_sz_y, pipette_bottom_holder_sz_z], center=true);
									}
									translate([0, 0, wall+pipette_bottom_holder_sz_z/2]) {
										cube([pipette_bottom_holder_sz_x+0.8, pipette_bottom_holder_sz_y+0.8, pipette_bottom_holder_sz_z], center=true);
									}
								}
								cube([pipette_bottom_holder_sz_x-2*pipette_wall, pipette_bottom_holder_sz_y-2*pipette_wall, pipette_bottom_holder_sz_z], center=true);
							}
						}
					}
				}
			}
			translate([0, -default_attach_extra_sz_y-(pipette_center_off_y-attach_sz_y/2-gantry_sz_y/2), -(-attach_sz_z/2+holder_sz_z/2+holder_off_z)]) {
				attachment_holes(d=hole_d_through, hole_count=20);
			}
		}
	} 
}

module pipette_top_holder() {
	translate([0, pipette_center_off_y-attach_sz_y/2-gantry_sz_y/2, 39]) {
		full_sz_z = attach_sz_z/2 + 9;
		difference() {
			union() {
				translate([0, pipette_center_off_y, 0]) {
					for(x=[-1,1]) {
						translate([x*(holder_sz_x+wall+1)/2, -pipette_center_off_y, full_sz_z]) {
							cube([wall, holder_sz_y/2, full_sz_z], center=true);
						}
						translate([0, 0, full_sz_z-pipette_top_holder_sz_z/2+full_sz_z/2]) {
							difference() {
								cube([holder_sz_x+2*wall+1, pipette_top_holder_sz_y+2*wall, pipette_top_holder_sz_z], center=true);
								hull() {
									translate([0, 0, 0]) {
										cube([pipette_top_holder_sz_x+0.8, pipette_top_holder_sz_y+0.8, pipette_top_holder_sz_z], center=true);
									}
								}
//								cube([pipette_top_holder_sz_x-2*pipette_wall, pipette_top_holder_sz_y-2*pipette_wall, pipette_top_holder_sz_z], center=true);
							}
						}
					}
				}



				translate([0, pipette_center_off_y, 0]) {
					at_h = full_sz_z;
					for(x=[-1,1]) {
						translate([x*(holder_sz_x+wall+1)/2, -pipette_center_off_y-5, at_h/2-pipette_top_holder_sz_z/2]) {
							difference() {
//								cube([wall, holder_sz_y/2+10, at_h], center=true);
							}
						}
					}
					translate([0, 0, 0]) {
//						cylinder(d=widest_level_d+2*wall, h=pipette_top_holder_sz_z, center=true);
						translate([0, 4, 0]) {
//							cube([holder_sz_x+2*wall+1, 20, pipette_top_holder_sz_z], center=true);
						}
					}
				}
				translate([0, -widest_level_d/2-20, 0]) {
//					cube([16, 20, pipette_top_holder_sz_z], center=true);
				}
			}
			translate([0, -default_attach_extra_sz_y-(pipette_center_off_y-attach_sz_y/2-gantry_sz_y/2), full_sz_z]) {
				attachment_holes(d=hole_d_through);
			}
			translate([0, pipette_center_off_y, 0]) {
				cylinder(d=widest_level_d+2*pipette_gap, h=pipette_top_holder_sz_z, center=true);
			}
			translate([0, -widest_level_d/2-20, 0]) {
//				cube([6, 20, pipette_top_holder_sz_z], center=true);
				translate([0, -5, 0]) {
					rotate([0, 90, 0]) {
						cylinder(d=hole_d_through, h=40, center=true);
						translate([0, 0, -16/2]) {
							nutHole(4);
						}
					}
				}
			}
		}
	} 
}

servo_sz_x = 20;
servo_sz_y = 38;
servo_sz_z = 40;
servo_sz_full_z = 55;
servo_hole_off_x = 5;
servo_hole_off_y = 9;
servo_hole_sz_y = 3;
servo_rolor_off_z = 10;
servo_rolor_d = 10;
servo_rolor_sz_y = 7;
servo_hole_dist_x = 9.5;
servo_hole_dist_z = 50;
servo_hole_d_through = 3.6;

module servo_1501MG_attach_holes() {
	for(z=[-1,1]) {
		for(x=[-1,1]) {
			translate([x*servo_hole_dist_x/2, 10, z*servo_hole_dist_z/2]) {
				rotate([90, 0, 0]) {
					cylinder(d=servo_hole_d_through, h=20, center=true);
				}
			}
		}
	}
}

module servo_1501MG_mockup() {
//	union() {
	difference() {
		union() {
			cube([servo_sz_x, servo_sz_y, servo_sz_z], center=true);
			translate([0, servo_sz_y/2-servo_hole_off_y, 0]) {
				cube([servo_sz_x, servo_hole_sz_y, servo_sz_full_z], center=true);
			}
			translate([0, (servo_sz_y+servo_rolor_sz_y)/2, servo_sz_z/2-servo_rolor_off_z]) {
				rotate([90, 0, 0]) {
					cylinder(d=servo_rolor_d, h=servo_rolor_sz_y, center=true);
				}
			}
		}
		servo_1501MG_attach_holes();
	}
}

servo_holder_wall = 5;
servo_hold_panel_z = 50;
servo_hold_panel_hole_step_count = 5;
servo_hold_panel_hole_step_sz = (servo_hold_panel_z-12)/servo_hold_panel_hole_step_count;

module push_servo_holder_plate() {
	translate([-wall, pipette_center_off_y-attach_sz_y/2-gantry_sz_y/2, 39]) {
		full_sz_z = attach_sz_z/2 + 9;
		difference() {
			union() {
				translate([0, 0, full_sz_z]) {
					for(x=[-1,1]) {
						translate([x*(holder_sz_x+wall+1)/2, 0, 0]) {
							translate([0, 0, 12.5]) {
								cube([wall, holder_sz_y/2, full_sz_z+25], center=true);
							}
							servo_holder_off_y = 26;
							translate([0, -servo_holder_off_y/2, full_sz_z/2-pipette_top_holder_sz_z/2]) {
								for(z=[0, 25]) {
									translate([0, 0, z]) {
										cube([wall, pipette_top_holder_sz_y+servo_holder_off_y, pipette_top_holder_sz_z], center=true);
									}
								}
								hull() {
									translate([0, 10, 0]) {
										cube([wall, wall, pipette_top_holder_sz_z], center=true);
									}
									translate([x*(servo_sz_x-wall)/2, -servo_holder_off_y+servo_holder_wall/2, 0]) {
										cube([servo_sz_x, servo_holder_wall, pipette_top_holder_sz_z], center=true);
									}
								}
								translate([x*(servo_sz_x-wall)/2, -servo_holder_off_y+servo_holder_wall/2, servo_hold_panel_z/2]) {
									difference() {
										cube([servo_sz_x, servo_holder_wall, servo_hold_panel_z], center=true);
										for(z=[1:1:servo_hold_panel_hole_step_count]) {
											translate([0, 0, 5-servo_hold_panel_z/2+z*servo_hold_panel_hole_step_sz]) {
												rotate([90, 0, 0]) {
													cylinder(d=hole_d, h=20, center=true);
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
			translate([0, -default_attach_extra_sz_y-(pipette_center_off_y-attach_sz_y/2-gantry_sz_y/2), full_sz_z]) {
				attachment_holes(d=hole_d_through, hole_count=10);
			}
		}
	}
}

module push_servo_holder() {
	translate([-33.5, -84, 135]) {
		servo_attach_sz_z = servo_sz_full_z+80;
		difference() {
//		union() {
			cube([servo_sz_x+6, servo_holder_wall, servo_attach_sz_z], center=true);
			for(z=[1:1:9]) {
				translate([0, 0, -servo_attach_sz_z/2+z*servo_hold_panel_hole_step_sz]) {
					hull() {
						for(x=[-1,1]) {
							translate([x*3, 0, 0]) {
								rotate([90, 0, 0]) {
									cylinder(d=hole_d, h=20, center=true);
								}
							}
						}
					}
				}
			}
			translate([0, -10, 39]) {
				cube([servo_sz_x+1, servo_sz_y, servo_sz_z+1], center=true);
				servo_1501MG_attach_holes();
			}
		}
	}	
}

gantry_z();
translate([0, 0, 30]) {
  pipette_top_holder();
}
//push_servo_holder();
translate([0, 0, 30]) {
  //push_servo_holder_plate();
}
translate([-33.5, -90, 174]) {
	%servo_1501MG_mockup();
}
pipette_bottom_holder();
//color("blue") {
//gantry_z_attachment(extra_sz_z=attach_sz_z+10);
%gantry_z_attachment_sides(extra_sz_y=default_attach_extra_sz_y, extra_sz_z=attach_sz_z+10+26);
//}

translate([-16, -70, -86]) {
	%pipette_mockup();
}
