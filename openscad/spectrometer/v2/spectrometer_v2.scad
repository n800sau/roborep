use <../../lib/ear.scad>

$fn = 36;

wall = 2;

grating_sz = 53;
grating_sz_z = 1.5;

grating_window_sz = 40;

tube_sz = 60;
tube_len = 120;

cam_block_tube_len = 30;

cam_pcb_sz_x = 4;
cam_pcb_sz_y = 25;
cam_pcb_sz_z = 24;

cam_hole_offset_z = 9.5;
cam_hole_dist_y = 21;
cam_hole_dist_z = 12;
cam_hole_d = 3;

cam_holder_sz_x = 30;
cam_holder_sz_z = grating_sz - cam_holder_sz_x / 2;

cam_wire_sz_z = 5;
cam_wire_sz_y = 18;

hole_d = 3;
hole_d_through = 4;

module ear(base_dist, height, d=3, cone=false) {
	bolt_hole_cone_center(base_dist=base_dist, hole_d=d, hole_wall=3, height=height, hole_height=cone ? height/2 : height);
}

module grating_insert() {
	difference() {
		union() {
			translate([0, 0, wall/2+grating_sz_z+wall]) {
				// top
				cube([grating_sz+2*wall, grating_sz+2*wall, wall], center=true);
			}
			translate([0, 0, wall+grating_sz_z/2]) {
				// middle
				cube([grating_sz+2*wall, grating_sz+2*wall, grating_sz_z], center=true);
			}
			translate([0, 0, wall/2]) {
				// bottom
				cube([grating_sz+2*wall, grating_sz+2*wall, wall], center=true);
				translate([0, -grating_sz/2, 8]) {
					cube([grating_sz-14, wall*2, 15], center=true);
				}
			}
		}
		cube([grating_window_sz, grating_window_sz, 50], center=true);
		translate([0, 0, wall+grating_sz_z/2]) {
			// middle
			translate([0, wall, 0]) {
				cube([grating_sz, grating_sz+wall, grating_sz_z], center=true);
			}
			cube([20, grating_sz+2*wall, grating_sz_z], center=true);
			translate([0, -grating_sz/2, 8]) {
				cube([grating_sz-22, wall*2, 4], center=true);
			}
		}
	}
}

module tube_block() {
	translate([0, 0, tube_sz/2+wall]) {
		translate([-4+wall/2, 0, -(tube_sz+wall)/2]) {
			difference() {
				// bottom
				cube([tube_len+wall+4, tube_sz+2*wall, wall], center=true);
				translate([54, 0, 0]) {
					cylinder(d=hole_d, h=20, center=true);
				}
			}
		}
		// side walls
		for(ysgn=[-1,1]) {
			difference() {
				translate([0, ysgn*(tube_sz+wall)/2, 0]) {
					cube([tube_len, wall, tube_sz], center=true);
					for(xsgn=[-1,1]) {
						translate([xsgn * 30, ysgn*-5, tube_sz/2]) {
							rotate([0, 0, ysgn*90]) {
								ear(0, 10, cone=true);
							}
						}
					}
				}
				translate([54, ysgn*(tube_sz+wall)/2, 0]) {
					rotate([ysgn*90, 0, 0]) {
						cylinder(d=hole_d, h=20, center=true);
					}
				}
			}
		}
		// slit end
		translate([-tube_len/2, 0, -wall/2]) {
			difference() {
				union() {
					cube([wall, tube_sz+2*wall, tube_sz+wall], center=true);
					translate([-4-wall, 0, 0]) {
						cube([wall, tube_sz+2*wall, tube_sz+wall], center=true);
					}
					for(ysgn=[-1,1]) {
						translate([-4+wall/2, ysgn*(tube_sz+wall)/2, 0]) {
							cube([4, wall, tube_sz+wall], center=true);
						}
					}
				}
				translate([0, 0, 10]) {
					cube([30, 45, 60], center=true);
				}
			}
		}
	}
}

module camera_block() {
	translate([-cam_block_tube_len/2, 0, tube_sz/2+wall]) {
		translate([0, 0, -(tube_sz+wall)/2]) {
			difference() {
				// bottom
				cube([cam_block_tube_len, tube_sz+2*wall, wall], center=true);
				translate([54, 0, 0]) {
					cylinder(d=hole_d, h=20, center=true);
				}
			}
		}
		// side walls
		for(ysgn=[-1,1]) {
			difference() {
				translate([tube_sz/2, ysgn*(tube_sz+wall)/2, 0]) {
					translate([ysgn>0 ? -tube_sz/2 : 0, ysgn*(tube_sz+wall)/2, 0]) {
						cube([cam_block_tube_len+(ysgn<0 ? tube_sz : 0), wall, tube_sz], center=true);
					}
					for(xsgn=ysgn>0 ? [-1 : 1] : [1]) {
						translate([xsgn * 30, ysgn*-5, tube_sz/2]) {
							rotate([0, 0, ysgn*90]) {
								ear(0, 10, cone=true);
							}
						}
					}
				}
				translate([54, ysgn*(tube_sz+wall)/2, 0]) {
					rotate([ysgn*90, 0, 0]) {
						cylinder(d=hole_d, h=20, center=true);
					}
				}
			}
		}
	}
	translate([0, -tube_sz/2-wall, 0]) {
		cube([60, 120, wall], centre=true);
	}
	translate([0, tube_sz/2, 0]) {
		cube([wall, 90-tube_sz/2, tube_sz+wall], centre=true);
	}
	translate([60, -tube_sz/2, 0]) {
		cube([wall, 90+tube_sz/2, tube_sz+wall], centre=true);
	}
}

module cam_holder_holes(d) {
	translate([-(box_sz_x-wall)/2+cam_wall_offset-2*wall-cam_pcb_sz_x-7, 0, 0]) {
		translate([-5, 0, -5]) {
			rotate([90, 0, 0]) {
				cylinder(d=d, h=2*box_sz_y, center=true);
			}
		}
	}
}

module cam_holder() {
	difference() {
		union() {
			translate([0, 0, wall/2]) {
				cube([cam_holder_sz_x+2*wall, cam_holder_sz_z+2*wall, wall], center=true);
				translate([0, -cam_holder_sz_z/2, 8]) {
					// bottom
					cube([cam_holder_sz_x, wall*2, 15], center=true);
				}
			}
			rotate([0, 90, 0]) {
				translate([cam_pcb_sz_x/2, 4, 0]) {
					translate([cam_pcb_sz_x, 0, 0]) {
						%cam_pcb_sub();
					}
				}
			}
		}
		translate([0, -cam_holder_sz_z/2, 10]) {
			// bottom hole
			cube([cam_holder_sz_x-10, wall*2, 4], center=true);
		}
		translate([0, 4, 0]) {
			rotate([0, 90, 0]) {
				cam_holes();
			}
		}
	}
}

module cam_holes(d=cam_hole_d, h=20) {
	for(ysgn=[-1, 1]) {
		translate([0, ysgn*cam_hole_dist_y/2, (-cam_pcb_sz_z)/2+cam_hole_offset_z]) {
			rotate([0, 90, 0]) {
				cylinder(d=d, h=h, center=true);
			}
			translate([0, 0, cam_hole_dist_z]) {
				rotate([0, 90, 0]) {
					cylinder(d=d, h=h, center=true);
				}
			}
		}
	}
}

module cam_pcb_sub() {
	difference() {
		cube([cam_pcb_sz_x, cam_pcb_sz_y, cam_pcb_sz_z], center=true);
		cam_holes();
	}
}

// -----------------------------------------------------

translate([15, -10, 0]) {
	rotate([0, 0, 45]) {
		translate([10, 0, 51/2+2*wall]) {
			rotate([90, 0, 90]) {
//				grating_insert();
			}
		}
	}
}

translate([-tube_len/2, 0, 0]) {
//	tube_block();
}

camera_block();

translate([30, 40, 24]) {
	rotate([90, 0, 180]) {
//		cam_holder();
	}
}

difference() {
//	cam_holes(d=cam_hole_d+2*wall, h=cam_pcb_sz_x);
//	cam_holes();
}
