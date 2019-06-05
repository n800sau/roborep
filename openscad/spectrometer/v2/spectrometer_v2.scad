use <../../lib/ear.scad>

$fn = 36;

wall = 2;
gap = 0.5;

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

slit_plate_sz_x = 4;

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
				// slot for bolt
				cube([grating_sz-22, wall*2, 4], center=true);
			}
		}
	}
}

module tube_block() {
	translate([0, 0, tube_sz/2+wall]) {
		translate([-slit_plate_sz_x+wall/2, 0, -(tube_sz+wall)/2]) {
			difference() {
				// bottom
				cube([tube_len+wall+slit_plate_sz_x, tube_sz+2*wall, wall], center=true);
// ATTENTION fix hole location here
				translate([tube_len/2-6, 0, 0]) {
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
				translate([tube_len/2-6, ysgn*(tube_sz+wall)/2, 0]) {
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
					translate([-slit_plate_sz_x-wall, 0, 0]) {
						cube([wall, tube_sz+2*wall, tube_sz+wall], center=true);
					}
					for(ysgn=[-1,1]) {
						translate([-slit_plate_sz_x+wall/2, ysgn*(tube_sz+wall)/2, 0]) {
							cube([slit_plate_sz_x, wall, tube_sz+wall], center=true);
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

module tube_block_top() {
	translate([0, 0, wall+tube_sz+wall/2]) {
		difference() {
			translate([(-wall-slit_plate_sz_x-wall-wall-gap/2)/2, 0, 0]) {
				cube([tube_len+wall+slit_plate_sz_x+wall/2+wall+gap, tube_sz+4*wall+2*gap, wall], center=true);
				// slit side parabrick
				translate([-(tube_len+wall+slit_plate_sz_x+wall/2+gap)/2, 0, -wall]) {
					cube([wall, tube_sz+2*wall+2*gap, wall], center=true);
				}
				// side parabrick
				for(ysgn=[-1,1]) {
					translate([0, ysgn*(tube_sz+3*wall+2*gap)/2, -wall]) {
						cube([tube_len+wall+slit_plate_sz_x+wall/2+wall+gap, wall, wall], center=true);
					}
				}
			}
			// side wall ear holes
			for(ysgn=[-1,1]) {
				for(xsgn=[-1,1]) {
					translate([xsgn * 30, ysgn*(tube_sz+wall)/2+ysgn*-5, 0]) {
						cylinder(d=hole_d_through, h=20, center=true);
					}
				}
			}
			translate([tube_len/2-6, 0, 0]) {
				cylinder(d=hole_d, h=20, center=true);
			}
		}
	}
}

module slit_plate() {
	translate([(-tube_len-wall-slit_plate_sz_x-gap)/2, gap/2, wall+(tube_sz-gap)/2]) {
		difference() {
			cube([slit_plate_sz_x-gap-0.5, tube_sz-gap-0.5, tube_sz-gap-0.5], center=true);
			cube([slit_plate_sz_x-gap, 0.5, 30], center=true);
		}
	}
}

module tube_and_camera_block_connector() {
	difference() {
		translate([-13.5, 0, 0]) {
color("blue")
			union() {
				translate([0, tube_sz/2+wall+gap, -wall-gap]) {
					// side
					cube([26, wall, tube_sz-gap]);
				}
				translate([0, -(tube_sz/2+2*wall+gap), -wall-gap]) {
					// side
					cube([26, wall, tube_sz-gap]);
				}
				translate([0, -tube_sz/2-wall-gap, -wall-gap]) {
					// bottom
					cube([26, tube_sz+2*wall+2*gap, wall]);
				}
			}
		}
		for(xsgn=[-1,1]) {
			for(ysgn=[-1,1]) {
				translate([0.5+xsgn*6.5, ysgn*(tube_sz+wall)/2, tube_sz/2+wall]) {
					rotate([90, 0, 0]) {
						cylinder(d=hole_d_through, h=20, center=true);
					}
				}
			}
			translate([-1+xsgn*8, 0, 0]) {
				cylinder(d=hole_d_through, h=20, center=true);
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
				translate([-6-wall, 0, 0]) {
					cylinder(d=hole_d, h=20, center=true);
				}
			}
		}
		// side walls
		for(ysgn=[-1,1]) {
			difference() {
				translate([tube_sz/2, ysgn*(tube_sz+wall)/2, 0]) {
					translate([ysgn>0 ? -tube_sz/2 : 0, 0, 0]) {
						cube([cam_block_tube_len+(ysgn<0 ? tube_sz : 0), wall, tube_sz], center=true);
					}
					for(xsgn=[-1,1]) {
						translate([5+xsgn * 30, ysgn*-5, tube_sz/2]) {
							rotate([0, 0, ysgn*90]) {
								if(!(ysgn>0 && xsgn>0)) {
									color("green") ear(0, 10, cone=true);
								}
							}
						}
					}
				}
				translate([-6-wall, ysgn*(tube_sz+wall)/2, 0]) {
					rotate([ysgn*90, 0, 0]) {
						cylinder(d=hole_d, h=20, center=true);
					}
				}
			}
		}
	}
	translate([0, -tube_sz/2-wall, 0]) {
		// bottom with holes
		difference() {
			cube([60, 120, wall]);
			// hole for grating
			translate([22, 34, 10]) {
				cylinder(d=20, h=20, center=true);
			}
			// hole for camera
			translate([31, 72, 10]) {
				cylinder(d=16, h=20, center=true);
			}
			translate([tube_sz/2, 120-6, 0]) {
				cylinder(d=hole_d, h=20, center=true);
			}
		}
	}
	translate([0, tube_sz/2, 0]) {
		// shorter side
		difference() {
			cube([wall, 90-tube_sz/2-wall, tube_sz+wall], centre=true);
			translate([0, 90-tube_sz/2-wall-6, wall+tube_sz/2]) {
				rotate([0, 90, 0]) {
					cylinder(d=hole_d, h=20, center=true);
				}
			}
		}
	}
	translate([60, -tube_sz/2-wall, 0]) {
		// longer side
		difference() {
			cube([wall, 90+tube_sz/2+wall, tube_sz+wall], centre=true);
			translate([0, 90+tube_sz/2+wall-6, wall+tube_sz/2]) {
				rotate([0, 90, 0]) {
					cylinder(d=hole_d, h=20, center=true);
				}
			}
		}
	}
	translate([60, -tube_sz/2-wall, 0]) {
		for(xsgn=[0,-1]) {
			for(ysgn=[-1]) {
				translate([-slit_plate_sz_x+xsgn*(tube_sz-10), 90+ysgn*-18, tube_sz+wall]) {
					rotate([0, 0, xsgn*180]) {
						color("green") ear(0, 10, cone=true);
					}
				}
			}
		}
	}
}

module camera_block_top() {
	translate([0, 0, tube_sz+wall+wall/2]) {
		difference() {
			union() {
				translate([-cam_block_tube_len/2, 0, 0]) {
					// bottom
					cube([cam_block_tube_len, tube_sz+4*wall+2*gap, wall], center=true);
					translate([-cam_block_tube_len/2, tube_sz/2+wall+gap, -wall/2-wall]) {
						cube([cam_block_tube_len-wall, wall, wall]);
					}
					translate([-cam_block_tube_len/2, -tube_sz/2-2*wall-gap, -wall/2-wall]) {
						cube([cam_block_tube_len+60+wall+2*gap, wall, wall]);
					}
				}
				translate([-wall-gap, -tube_sz/2-2*wall-gap, -wall/2]) {
					// left bottom
					cube([60+wall+2*wall+2*gap, 120+wall+gap, wall]);
					translate([0, 65+wall, -wall]) {
						cube([wall, 55+gap, wall]);
					}
					translate([tube_sz+2*wall+2*gap, 0, -wall]) {
						cube([wall, 120+wall+gap, wall]);
					}
				}
			}
			translate([-cam_block_tube_len/2, 0, 0]) {
				translate([-6-wall, 0, 0]) {
					cylinder(d=hole_d, h=20, center=true);
				}
				for(ysgn=[-1,1]) {
					translate([tube_sz/2, ysgn*(tube_sz+wall)/2, 0]) {
						for(xsgn=[-1,1]) {
							translate([5+xsgn * 30, ysgn*-5, 0]) {
								if(!(ysgn>0 && xsgn>0)) {
									cylinder(d=hole_d_through, h=30, center=true);
								}
							}
						}
					}
				}
			}
			translate([0, -tube_sz/2-wall, -wall/2]) {
				translate([tube_sz/2, 120-6, 0]) {
					cylinder(d=hole_d, h=20, center=true);
				}
				// out hole holes
				translate([60, 0, 0]) {
					for(xsgn=[0,-1]) {
						for(ysgn=[-1]) {
							translate([-slit_plate_sz_x+xsgn*(tube_sz-10), 90+ysgn*-18, 0]) {
								cylinder(d=hole_d_through, h=20,center=true);
							}
						}
					}
				}
			}
		}
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

module oval_washer(ext_d, d, h, dist) {
	difference() {
		hull() {
			cylinder(d=ext_d, h=h);
			translate([0, dist, 0]) {
				cylinder(d=ext_d, h=h);
			}
		}
		hull() {
			cylinder(d=d, h=h);
			translate([0, dist, 0]) {
				cylinder(d=d, h=h);
			}
		}
	}
}


// -----------------------------------------------------

translate([-tube_len/2, 0, 0]) {
	color("green")	tube_block();
	//slit_plate();
	tube_block_top();
}

translate([cam_block_tube_len, 0, 0]) {
//	camera_block();
	//camera_block_top();
	translate([22, 3, 0]) {
		rotate([0, 0, $t*360]) {
			translate([-10, 0, 51/2+2*wall]) {
				rotate([90, 0, 90]) {
//				 color("blue") grating_insert();
				}
			}
			%cylinder(d=5, h=50, center=true);
			translate([0, 0, -5]) {
//				oval_washer(ext_d=25, d=4, h=wall, dist=10);
			}
		}
	}

	translate([31, 40, 24]) {
		rotate([0, 0, $t*360]) {
			translate([0, 0, -25]) {
				%cylinder(d=5, h=50, center=true);
				translate([0, 0, -4]) {
//					oval_washer(ext_d=25, d=4, h=wall, dist=10);
				}
			}
			// set center in slot center
			translate([0, -10, 0]) {
				rotate([90, 0, 180]) {
//			color("blue") cam_holder();
				}
			}
		}
	}
}

translate([0, 0, 0]) {
//	tube_and_camera_block_connector();
}

difference() {
//	cam_holes(d=cam_hole_d+2*wall, h=cam_pcb_sz_x);
//	cam_holes();
}

