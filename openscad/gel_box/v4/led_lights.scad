$fn = 50;

use <MCAD/2Dshapes.scad>
use <pcb_led_cvf.scad>
use <../../lib/ear.scad>

wall = 2;
hole_d = 3.2;
hole_d_through = 4;

hole_dist_x = 64;
hole_dist_y = 42.5;

hole_count_x = 24;
hole_count_y = 18;

pcb_sz_x = 70;
pcb_sz_y = 50.5;
pcb_sz_z = 1.2;
pcb_hole_dist = 2.54;
pcb_first_hole_x = (pcb_sz_x - pcb_hole_dist * (hole_count_x-1))/2;
pcb_first_hole_y = (pcb_sz_y - pcb_hole_dist * (hole_count_y-1))/2;
//pcb_first_hole_x = 0;
//pcb_first_hole_y = 0;
//echo("pcb_first_hole_x=", pcb_first_hole_x);
//echo("pcb_first_hole_y=", pcb_first_hole_y);

hole_per_led_x = 3;
hole_per_led_y = 3;

led_count_x = hole_count_x / hole_per_led_x;
led_count_y = hole_count_y / hole_per_led_y;

led_step_x = pcb_hole_dist * hole_per_led_x;
led_step_y = pcb_hole_dist * hole_per_led_y;

led_d = 5.5;
led_leg_sz = 16;
led_holder_sz_z = min(4, led_leg_sz - pcb_sz_z - wall - 4);

//echo("led_holder_sz_z=", led_holder_sz_z);

led_leg_encircle = 3.1;
led_holder_wall = 1;

holder_skirt_sz = 8;
pcb_holder_plate_sz_x = pcb_sz_x + 2 * holder_skirt_sz;
pcb_holder_plate_sz_y = pcb_sz_y + 2 * holder_skirt_sz;
pcb_holder_plate_sz_z = wall;

pcb_inter_gap = 0.5;
full_size_pcb_holder_plate_sz_y = 2 * pcb_sz_y + pcb_inter_gap + 2 * holder_skirt_sz;

stepdown_pcb_sz_x = 48.3;
stepdown_pcb_sz_y = 24;
stepdown_pcb_sz_z = 1.1;;

stepdown_holder_sz_x = pcb_sz_x - wall;
stepdown_holder_sz_y = full_size_pcb_holder_plate_sz_y - 2 * holder_skirt_sz - 2 * wall;
stepdown_holder_sz_z = 17;

power12_d = 8.4;

acryl_sz_x = 90;
acryl_sz_y = 118; //?
acryl_sz_z = 3.2;

acryl_gap = 0.5;
acryl_holder_sz_x = max(pcb_holder_plate_sz_x, acryl_sz_x + acryl_gap);
acryl_holder_sz_y = max(full_size_pcb_holder_plate_sz_y, acryl_sz_y + acryl_gap);
acryl_holder_sz_z = 17;

module a_led_holder(with_hole=true) {
	difference() {
		if(with_hole) {
			cylinder(d=led_leg_encircle+2*led_holder_wall, h=led_holder_sz_z, center=true);
		}
		cylinder(d=led_leg_encircle, h=led_holder_sz_z, center=true);
	}
}

module switch_holes() {
	cube([9.5, 5, 50], center=true);
	translate([9.5, 0, 0]) {
		cylinder(d=2.5, h = 50, center=true);
	}
	translate([-9.5, 0, 0]) {
		cylinder(d=2.5, h = 50, center=true);
	}
}

module sandwich_holes(d, h=50, arc=false) {
	// sandwich holes
	for(xsgn=[-1, 0, 1]) {
		for(ysgn=[-1, -0.5, 0, 0.5, 1]) {
			if(abs(xsgn)==1 || abs(ysgn)==1) {
				translate([xsgn*(pcb_holder_plate_sz_x-holder_skirt_sz)/2, ysgn*(full_size_pcb_holder_plate_sz_y-holder_skirt_sz)/2, 0]) {
					if(arc) {
						translate([0, 0, -h/2]) {
							if(abs(xsgn) == 1 && abs(ysgn) == 1) {
								linear_extrude(height = h) {
									angle = xsgn > 0 ? (ysgn > 0 ? -135 : 135) : (ysgn > 0 ? -45 : 45);
									rotate([0, 0, angle]) {
										donutSlice(d-wall, d, -90, 90);
									}
								}
							} else if(abs(xsgn) == 1) {
								linear_extrude(height = h) {
									angle = xsgn > 0 ? 180 : 0;
									rotate([0, 0, angle]) {
										donutSlice(d-wall, d, -90, 90);
									}
								}
							} else {
								if(abs(ysgn) != 0) {
									linear_extrude(height = h) {
										angle = ysgn > 0 ? -90 : 90;
										rotate([0, 0, angle]) {
											donutSlice(d-wall, d, -90, 90);
										}
									}
								}
							}
						}
					} else {
						cylinder(d=d, h=h, center=true);
					}
				}
			}
		}
	}
}

module ear(base_dist, height) {
  bolt_hole_cone_center(base_dist=base_dist, hole_d=3, hole_wall=3, height=height, hole_height=height);
}

module ear_holes(d, h=50, base_dist=3, ear_mode=false, height=7) {
	// sandwich holes
	for(xsgn=[-1, 1]) {
		for(ysgn=[-1, 1]) {
			if(abs(xsgn)==1 || abs(ysgn)==1) {
				translate([xsgn*(pcb_holder_plate_sz_x-holder_skirt_sz)/2, ysgn*(full_size_pcb_holder_plate_sz_y-holder_skirt_sz)/2, 0]) {
					if(ear_mode) {
            if(abs(xsgn) == 1) {
              angle = xsgn > 0 ? 0 : 180;
              rotate([180, 0, angle]) {
                ear(base_dist=base_dist, height=height);
              }
            } else {
              if(abs(ysgn) != 0) {
                angle = ysgn > 0 ? 90 : -90;
                rotate([180, 0, angle]) {
                  ear(base_dist=base_dist, height=height);
                }
							}
						}
					} else {
						cylinder(d=d, h=h, center=true);
					}
				}
			}
		}
	}
}

module pcb_holder_plate() {
	difference() {
		for(ygs1=[-1,1]) {
			translate([0, ygs1*(pcb_sz_y/2+0.25), 0]) {
				cube([pcb_holder_plate_sz_x, pcb_holder_plate_sz_y, pcb_holder_plate_sz_z], center=true);
				for(x=[0:led_count_x-1]) {
					for(y=[0:led_count_y-1]) {
						translate([pcb_first_hole_x+x*led_step_x-pcb_sz_x/2+pcb_hole_dist/2, pcb_first_hole_y+y*led_step_y-pcb_sz_y/2+pcb_hole_dist, (led_holder_sz_z+pcb_holder_plate_sz_z)/2]) {
							a_led_holder(with_hole=true);
						}
					}
				}
			}
		}
		for(ygs2=[-1,1]) {
			translate([0, ygs2*(pcb_sz_y/2+0.25), 0]) {
				// holes in pcb area (for preview only)
				%for(x=[0:hole_count_x-1]) {
					for(y=[0:hole_count_y-1]) {
						translate([pcb_first_hole_x+x*pcb_hole_dist-pcb_sz_x/2, pcb_first_hole_y+y*pcb_hole_dist-pcb_sz_y/2, 0]) {
				//			cylinder(d=2, h=50, center=true);
						}
					}
				}
				for(x=[0:led_count_x-1]) {
					for(y=[0:led_count_y-1]) {
						translate([pcb_first_hole_x+x*led_step_x-pcb_sz_x/2+pcb_hole_dist/2, pcb_first_hole_y+y*led_step_y-pcb_sz_y/2+pcb_hole_dist, 0]) {
							a_led_holder(with_hole=false);
						}
					}
				}
				// holes in pcb
				for(xsgn=[-1,1]) {
					for(ysgn=[-1,1]) {
						translate([xsgn*hole_dist_x/2, ysgn*hole_dist_y/2, 0]) {
							cylinder(d=2, h=50, center=true);
						}
					}
				}
			}
		}
		ear_holes(d=hole_d);
	}
}

module stepdown_holder_v1() {
	difference() {
		union() {
			for(ysgn=[-1,1]) {
				translate([0, ysgn*full_size_pcb_holder_plate_sz_y/2, 0]) {
					cube([pcb_holder_plate_sz_x+2*wall, wall, stepdown_holder_sz_z], center=true);
				}
			}
			for(xsgn=[-1,1]) {
				translate([xsgn*pcb_holder_plate_sz_x/2, 0, 0]) {
					cube([wall, full_size_pcb_holder_plate_sz_y+2*wall, stepdown_holder_sz_z], center=true);
				}
			}
			translate([0, 0, -stepdown_holder_sz_z/2]) {
				cube([pcb_holder_plate_sz_x+2*wall, full_size_pcb_holder_plate_sz_y+2*wall, wall], center=true);
			}
		}
		ear_holes(d=hole_d+wall*4);
	}
	translate([0, 0, -wall/2]) {
		ear_holes(d=hole_d+wall, h=stepdown_holder_sz_z);
	}
	translate([0, 0, stepdown_holder_sz_z/2]) {
		ear_holes(d=hole_d+wall*4, h=wall);
	}
}

module stepdown_holder() {
	difference() {
		union() {
			for(ysgn=[-1,1]) {
				// long ends
				translate([0, ysgn*(stepdown_holder_sz_y+wall)/2, 0]) {
					cube([stepdown_holder_sz_x, wall, stepdown_holder_sz_z], center=true);
					translate([0, ysgn*(holder_skirt_sz+wall)/2, (stepdown_holder_sz_z-wall)/2]) {
						cube([stepdown_holder_sz_x+2*wall, holder_skirt_sz, wall], center=true);
					}
				}
			}
			for(xsgn=[-1,1]) {
				// short ends
				translate([xsgn*(stepdown_holder_sz_x+wall)/2, 0, 0]) {
					cube([wall, stepdown_holder_sz_y+2*wall, stepdown_holder_sz_z], center=true);
					translate([xsgn*(holder_skirt_sz+wall)/2, 0, (stepdown_holder_sz_z-wall)/2]) {
						cube([holder_skirt_sz, 2*holder_skirt_sz+stepdown_holder_sz_y+2*wall, wall], center=true);
					}
				}
			}
			translate([0, 0, -stepdown_holder_sz_z/2]) {
				cube([stepdown_holder_sz_x+2*wall, stepdown_holder_sz_y+2*wall, wall], center=true);
			}
			translate([-25, -40 , -stepdown_holder_sz_z/2]) {
				pcb_led_cvf();
			}
		}
		ear_holes(d=hole_d_through);
		// 12v power hole
		translate([-19, stepdown_holder_sz_y-25, 0]) {
			rotate([90, 0, 0]) {
				cylinder(d=power12_d, h=50);
			}
		}
		translate([10, stepdown_holder_sz_y-50, 0]) {
			rotate([90, 0, 0]) {
				switch_holes();
			}
		}
	}
}

module acryl_holder() {
	difference() {
		union() {
			for(ysgn=[-1,1]) {
				// long ends
				translate([0, ysgn*(acryl_holder_sz_y+wall)/2, 0]) {
					cube([acryl_holder_sz_x+2*wall, wall, acryl_holder_sz_z], center=true);
          translate([0, -ysgn*wall, -acryl_sz_z/2]) {
            cube([acryl_holder_sz_x+2*wall, wall, acryl_holder_sz_z-acryl_sz_z], center=true);
          }
					translate([0, -ysgn*(acryl_holder_sz_y-stepdown_holder_sz_y)/4, -(acryl_holder_sz_z-wall)/2]) {
						cube([acryl_holder_sz_x+2*wall, (acryl_holder_sz_y-stepdown_holder_sz_y)/2, wall], center=true);
					}
				}
			}
			for(xsgn=[-1,1]) {
				// short ends
				translate([xsgn*(acryl_holder_sz_x+wall)/2, 0, 0]) {
					cube([wall, acryl_holder_sz_y+2*wall, acryl_holder_sz_z], center=true);
          translate([-xsgn*wall, 0, -acryl_sz_z/2]) {
            cube([wall, acryl_holder_sz_y+2*wall, acryl_holder_sz_z-acryl_sz_z], center=true);
          }
					translate([-xsgn*(acryl_holder_sz_x-stepdown_holder_sz_x)/4, 0, -(acryl_holder_sz_z-wall)/2]) {
						cube([(acryl_holder_sz_x-stepdown_holder_sz_x)/2, acryl_holder_sz_y+2*wall, wall], center=true);
					}
				}
			}
			translate([0, 0, (acryl_holder_sz_z-acryl_sz_z)/2]) {
				%cube([acryl_sz_x, acryl_sz_y, acryl_sz_z], center=true);
			}
		}
		ear_holes(d=hole_d);
	}
	translate([0, 0, -(acryl_holder_sz_z-acryl_sz_z)/2]) {
    ear_holes(d=hole_d, ear_mode=true, height=acryl_holder_sz_z-acryl_sz_z-wall);
  }
}

module diffuser(x_count, y_count, step_x, step_y) {
	sz_x = x_count*step_x;
	sz_y = y_count*step_y;
	sz_z = max(step_x, step_y);
	difference() {
		union() {
			translate([-sz_x/2, -sz_y/2, 0]) {
				for(x=[0:x_count]) {
					for(y=[0:y_count]) {
						translate([x*step_x, y*step_y, 0]) {
							sphere(d=max(step_x, step_y), center=true);
						}
					}
				}
			}
			translate([0, 0, 0.5]) {
				cube([sz_x, sz_y, 1], center=true);
			}
		}
		translate([0, 0, -sz_z/2]) {
			cube([sz_x*2, sz_y*2, sz_z], center=true);
		}
	}
}

module led_diffuser() {
	$fn = 10;
	d = max(led_step_x, led_step_y);
	for(ygs1=[-1,1]) {
		translate([0, ygs1*(pcb_sz_y-wall)/2, 0]) {
			cube([pcb_sz_x-wall, pcb_sz_y-wall, wall], center=true);
			for(x=[0:led_count_x-1]) {
				for(y=[0:led_count_y-1]) {
					translate([pcb_first_hole_x+x*led_step_x-pcb_sz_x/2+pcb_hole_dist/2, pcb_first_hole_y+y*led_step_y-pcb_sz_y/2+pcb_hole_dist, 0]) {
						difference() {
							sphere(d=d, center=true);
							translate([0, 0, -d/2]) {
								cube([d, d, d], center=true);
							}
						}
					}
				}
			}
		}
	}
}

//diffuser(10, 10, pcb_hole_dist, pcb_hole_dist);
translate([0, 0, acryl_holder_sz_z]) {
	led_diffuser();
}

translate([0, 0, acryl_holder_sz_z]) {
//	acryl_holder();
}
//pcb_holder_plate();
translate([0, 0, -stepdown_holder_sz_z]) {
//	stepdown_holder();
}
