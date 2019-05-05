show_side_plates = 1;
show_mask_plate = 0;
show_acryl_holder = 0;
show_led_holder_plate = 0;
show_stepdown_holder = 0;

$fn = 20;

use <MCAD/2Dshapes.scad>
use <../../lib/pcb_led_cvf.scad>
use <../../lib/ear.scad>

wall = 2;
gap = 0.25;
gaps = 2 * gap;
hole_d = 3.2;
hole_d_through = 4;

led_count_x = 10;
led_count_y = 10;

led_d = 5.5;
led_leg_sz = 16;
led_holder_sz_z = 1;

led_step_x = led_d + 1.5;
led_step_y = led_step_x;

led_leg_encircle = 3.1;
led_holder_wall = 1;

holder_skirt_sz = 8;
led_total_sz_x = led_step_x * led_count_x;
led_total_sz_y = led_step_y * led_count_y;
led_holder_plate_sz_x = led_total_sz_x + 2 * holder_skirt_sz;
led_holder_plate_sz_y = led_total_sz_y + 2 * holder_skirt_sz;
led_holder_plate_sz_z = wall;

stepdown_pcb_sz_x = 48.1;
stepdown_pcb_sz_y = 24;
stepdown_pcb_sz_z = 1.1;

stepdown_holder_sz_x = led_holder_plate_sz_x - 2 * holder_skirt_sz + 2 * wall;
stepdown_holder_sz_y = led_holder_plate_sz_y - 2 * holder_skirt_sz + 2 * wall;
stepdown_holder_sz_z = 37;
stepdown_holder_holes_z = 1.5+wall;

power12_d = 8.4;

acryl_sz_x = 90;
acryl_sz_y = 90; //?
acryl_sz_z = 3.2;

acryl_gap = 0.5;
acryl_holder_sz_x = max(led_holder_plate_sz_x, acryl_sz_x + acryl_gap);
acryl_holder_sz_y = max(led_holder_plate_sz_y, acryl_sz_y + acryl_gap);
acryl_holder_sz_z = 40;

top_cover_ear_sz_x = 10;
top_cover_ear_sz_y = 3;
top_cover_ear_sz_z = 40;
top_cover_ear_below_sz_z = 14;
top_cover_ear_sideplate_sz_z = 10;
top_cover_ear_hole_d = 4.5;

module a_led_holder() {
	difference() {
		cylinder(d=led_leg_encircle+2*led_holder_wall, h=led_holder_sz_z, center=true);
		difference() {
			cylinder(d=led_leg_encircle, h=led_holder_sz_z+100, center=true);
			cube([1, led_leg_encircle+2*led_holder_wall, led_holder_sz_z], center=true);
		}
	}
}

module a_led_hole() {
		difference() {
			cylinder(d=led_leg_encircle, h=led_holder_sz_z+100, center=true);
			cube([1, led_leg_encircle, led_holder_sz_z+100], center=true);
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

module ear(base_dist, height, d=3, cone=false) {
	bolt_hole_cone_center(base_dist=base_dist, hole_d=d, hole_wall=3, height=height, hole_height=cone ? height/2 : height);
}

module ear_holes(d, h=50, base_dist=3, ear_mode=false, cone=false, height=7) {
	for(xsgn=[-1, 1]) {
		for(ysgn=[-1, 1]) {
			translate([xsgn*(led_holder_plate_sz_x-holder_skirt_sz)/2, ysgn*(led_holder_plate_sz_y-holder_skirt_sz)/2, 0]) {
				if(ear_mode) {
					angle = cone ?
						(xsgn > 0 ? 
							(
								(ysgn > 0) ? -135 : 135
							) :
								(ysgn > 0) ? -45 : 45
						) :
						(xsgn > 0 ? 0 : 180);
					rotate([cone ? 0 : 180, 0, angle]) {
						ear(base_dist=base_dist, height=height, d=d, cone=cone);
					}
				} else {
					cylinder(d=d, h=h, center=true);
				}
			}
		}
	}
}

module led_holder_plate() {
	difference() {
		union() {
			difference() {
				cube([led_holder_plate_sz_x, led_holder_plate_sz_y, led_holder_plate_sz_z], center=true);
				for(x=[0:led_count_x-1]) {
					for(y=[0:led_count_y-1]) {
						translate([-led_holder_plate_sz_x/2+holder_skirt_sz+x*led_step_x+led_step_x/2, -led_holder_plate_sz_y/2+holder_skirt_sz+y*led_step_y+led_step_y/2, (led_holder_sz_z+led_holder_plate_sz_z)/2]) {
							a_led_hole();
						}
					}
				}
			}
			for(x=[0:led_count_x-1]) {
				for(y=[0:led_count_y-1]) {
					translate([-led_holder_plate_sz_x/2+holder_skirt_sz+x*led_step_x+led_step_x/2, -led_holder_plate_sz_y/2+holder_skirt_sz+y*led_step_y+led_step_y/2, (led_holder_sz_z+led_holder_plate_sz_z)/2]) {
						a_led_holder();
					}
				}
			}
		}
		ear_holes(d=hole_d);
	}
}

module stepdown_holder() {
	difference() {
		union() {
			for(ysgn=[-1,1]) {
				// long ends
				translate([0, ysgn*(stepdown_holder_sz_y+wall)/2, 0]) {
					cube([stepdown_holder_sz_x-6, wall, stepdown_holder_sz_z], center=true);
				}
			}
			for(xsgn=[-1,1]) {
				// short ends
				translate([xsgn*(stepdown_holder_sz_x+wall)/2, 0, 0]) {
					cube([wall, stepdown_holder_sz_y-6, stepdown_holder_sz_z], center=true);
          for(yside=[-1,1]) {
            translate([-xsgn*3, yside*(stepdown_holder_sz_y/2-2), 0]) {
              rotate([0, 0, xsgn*yside*45]) {
                cube([wall, 9, stepdown_holder_sz_z], center=true);
              }
						}
					}
				}
			}
			translate([0, 0, -stepdown_holder_sz_z/2]) {
				cube([stepdown_holder_sz_x+2*wall, stepdown_holder_sz_y+2*wall, wall], center=true);
			}
			translate([-25, -18.5 , -stepdown_holder_sz_z/2]) {
				rotate([90, 0, 0]) {
					pcb_led_cvf(
						pcb_width = stepdown_pcb_sz_x,
						pcb_length = stepdown_pcb_sz_y,
						pcb_height = stepdown_pcb_sz_z,
						add_tabs = false
					);
				}
			}
		}
			for(xsgn=[-1,1]) {
				// short ends
				translate([xsgn*(stepdown_holder_sz_x+wall)/2, 0, -9]) {
          for(yside=[-1,1]) {
            translate([xsgn*2, yside*(stepdown_holder_sz_y/2+1), 0]) {
              rotate([0, 0, xsgn*yside*45]) {
                cube([10, 20, stepdown_holder_sz_z], center=true);
              }
						}
					}
				}
			}
    
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
		// holes to adjust current
		for(xpos=[-23+12.5/*+33.5, 0, 20*/]) {
			translate([xpos, -stepdown_holder_sz_y/2+10, -stepdown_holder_sz_z/2+stepdown_holder_holes_z]) {
				rotate([90, 0, 0]) {
					cylinder(d=6, h=20);
				}
			}
		}
	}
	translate([0, 0, stepdown_holder_sz_z/2]) {
		ear_holes(d=hole_d_through, ear_mode=true, base_dist=1, cone=true, height=10);
	}
}

module side_holes(d=hole_d_through) {
  for(ysgn=[-1,1]) {
    for(xsgn=[-1,1]) {
      translate([xsgn*(acryl_holder_sz_y-30)/2, ysgn*(acryl_holder_sz_y+wall)/2, acryl_holder_sz_z/2-wall-acryl_sz_z-5]) {
        rotate([90, 0, 0]) {
          cylinder(d=d, h=10, center=true);
        }
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
					translate([0, -ysgn*wall, -(wall+acryl_sz_z)/2]) {
						cube([acryl_holder_sz_x+2*wall, wall, acryl_holder_sz_z-acryl_sz_z-wall], center=true);
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
					translate([-xsgn*wall, 0, -(wall+acryl_sz_z)/2]) {
						cube([wall, acryl_holder_sz_y+2*wall, acryl_holder_sz_z-acryl_sz_z-wall], center=true);
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
    // side holes
    side_holes(d=hole_d);
	}
	translate([0, 0, (-acryl_holder_sz_z+wall-wall)/2]) {
		ear_holes(d=hole_d, ear_mode=true, height=acryl_holder_sz_z-acryl_sz_z-wall);
	}
}

module side_plates() {
  difference() {
    union() {
      translate([(acryl_holder_sz_x+top_cover_ear_sz_x)/2+wall+gap, 0, acryl_holder_sz_z/2-top_cover_ear_below_sz_z]) {
        cube([top_cover_ear_sz_x, acryl_holder_sz_y+2*top_cover_ear_sz_y+gaps+2*wall, wall], center=true);
      }
      translate([top_cover_ear_sz_x, 0, acryl_holder_sz_z-top_cover_ear_below_sz_z]) {
        for(ysgn=[-1,1]) {
          translate([(acryl_holder_sz_x-top_cover_ear_sz_x)/2+wall, ysgn*(acryl_holder_sz_y+2*wall+2*top_cover_ear_sz_y+gaps-top_cover_ear_sz_y)/2, 0]) {
            for(yoff=[0, -3*top_cover_ear_sz_y]) {
              translate([0, ysgn*yoff, 0]) {
                difference() {
                  union() {
                    translate([0, 0, -top_cover_ear_sz_z/4]) {
                      cube([top_cover_ear_sz_x, top_cover_ear_sz_y, top_cover_ear_sz_z/2], center=true);
                    }
                    if(yoff==0) {
                      // side panel
                      translate([(top_cover_ear_sz_x-acryl_holder_sz_x)/2-wall, 0, (top_cover_ear_sideplate_sz_z-top_cover_ear_sz_z-wall)/2]) {
                        cube([acryl_holder_sz_x-10, top_cover_ear_sz_y, top_cover_ear_sideplate_sz_z], center=true);
                        translate([(-acryl_holder_sz_x+10+wall)/2, 0, 0]) {
                          rotate([90, 0, 0]) {
                            cylinder(d=top_cover_ear_sideplate_sz_z, h=top_cover_ear_sz_y, center=true);
                          }
                        }
                      }
                    }
                    rotate([90, 0, 0]) {
                      cylinder(d=top_cover_ear_sz_x, h=top_cover_ear_sz_y, center=true);
                    }
                  }
                  rotate([90, 0, 0]) {
                    cylinder(d=top_cover_ear_hole_d, h=top_cover_ear_sz_y, center=true);
                  }
                }
              }
            }
          }
        }
      }
    }
    side_holes();
  }
}

module mask_plate() {
  difference() {
    cube([acryl_holder_sz_x-gaps, acryl_holder_sz_y-gaps, wall], center=true);
    cube([led_total_sz_x, led_total_sz_y, wall], center=true);
    ear_holes(d=hole_d_through);
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
	scale([0.99, 0.99, 1]) {
		difference() {
				cube([acryl_holder_sz_x-2*wall, acryl_holder_sz_y-2*wall, 1], center=true);
				translate([0, 0, -(acryl_holder_sz_z-acryl_sz_z)/2]) {
					ear_holes(d=hole_d);
					ear_holes(d=hole_d, ear_mode=true, height=acryl_holder_sz_z-acryl_sz_z-wall);
				}
		}
	}
	for(ygs1=[-1,1]) {
		translate([0, ygs1*(pcb_sz_y-wall)/2, 0]) {

//			cube([pcb_sz_x-wall, pcb_sz_y-wall, 1], center=true);
			for(x=[0:led_count_x-1]) {
				for(y=[0:led_count_y-1]) {
					translate([pcb_first_hole_x+x*led_step_x-pcb_sz_x/2+pcb_hole_dist/2, pcb_first_hole_y+y*led_step_y-pcb_sz_y/2+pcb_hole_dist, 0]) {
						difference() {
							scale([1, 1, 0.6]) {
								sphere(d=d, center=true);
							}
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
//union() {
difference() {
//	led_diffuser();
	translate([40, 0, 0]) {
//		cube([100, 100, 100], center=true);
	}
	translate([0, 40, 0]) {
//		cube([100, 100, 100], center=true);
	}
}
}

translate([0, 0, acryl_holder_sz_z]) {
  if(show_side_plates) {
    side_plates();
  }
  translate([0, 0, (acryl_holder_sz_z-wall-acryl_sz_z)/2]) {
    if(show_mask_plate) {
      mask_plate();
    }
  }
  if(show_acryl_holder) {
    acryl_holder();
  }
}
if(show_led_holder_plate) {
  led_holder_plate();
}
translate([0, 0, -stepdown_holder_sz_z]) {
  if(show_stepdown_holder) {
    stepdown_holder();
  }
}
