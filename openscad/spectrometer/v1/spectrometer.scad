$fn = 50;

use <../../lib/ear.scad>
use <MCAD/triangles.scad>

wall = 2;
gap = 0.5;

box_sz_x = 140;
box_sz_y = 40;
box_sz_z = 40;

window_sz_y = 15;
window_sz_z = 10;

cam_wall_offset = 30;
cam_wall_angle = 45;

cam_wall_sz_z = (box_sz_z - 2 * wall) / cos(cam_wall_angle);
cam_wall_sz_x = cam_wall_sz_z * sin(cam_wall_angle);

cam_pcb_sz_x = 4;
cam_pcb_sz_y = 25;
cam_pcb_sz_z = 24;

cam_window_sz_y = box_sz_y - 16;
cam_window_sz_z = cam_wall_sz_z - 16;

cam_hole_offset_z = 9.5;
cam_bottom_reserve = 10;
cam_hole_dist_y = 21;
cam_hole_dist_z = 12;
cam_hole_d = 2.5;

cam_wire_sz_z = 1;
cam_wire_sz_y = 16;

middle_insert_hole_sz_x = wall + gap;
middle_wall_offset = wall + 17 + wall + middle_insert_hole_sz_x/2;

slot_sz_y = box_sz_y - 8 - 2 * wall;
slot_sz_z = 0.5;

hole_d = 3;
hole_d_through = 4;

module ear_holes(flat_ear=false) {
	for(ysgn = [-1, 1]) {
		translate([0, ysgn*(box_sz_y-wall)/2, 0]) {
      for(xpos=[-(box_sz_x-40)/2, (box_sz_x-70)/2]) {
        translate([xpos, ysgn*4, box_sz_z/2]) {
          rotate([0, 0, ysgn*-90]) {
            if(flat_ear) {
              bolt_hole_cone_center(base_dist=0, hole_d=hole_d, height=wall, hole_height=wall);
            } else {
              bolt_hole_cone_center(base_dist=0, hole_d=hole_d);
            }
          }
        }
      }
    }
  }
}

module sp_base() {
	// bottom
	translate([0, 0, -(box_sz_z-wall)/2]) {
		cube([box_sz_x, box_sz_y, wall], center=true);
	}
	// side walls
  difference() {
    for(ysgn = [-1, 1]) {
      translate([0, ysgn*(box_sz_y-wall)/2, 0]) {
        cube([box_sz_x, wall, box_sz_z], center=true);
      }
    }
    cam_holder_holes(d=hole_d_through);
    grate_holder_holes(d=hole_d_through);
  }
  ear_holes();
	// window wall
	translate([(box_sz_x-wall)/2, 0, 0]) {
		difference() {
			cube([wall, box_sz_y, box_sz_z], center=true);
			cube([wall, window_sz_y, window_sz_z], center=true);
		}
	}
	// middle slot insert holder
	translate([(box_sz_x-wall)/2-middle_wall_offset, 0, 0]) {
		for(xsgn=[-1, 1]) {
			translate([xsgn*(middle_insert_hole_sz_x+wall)/2, 0, 0]) {
				difference() {
					cube([wall, box_sz_y, box_sz_z], center=true);
					cube([wall, window_sz_y, window_sz_z], center=true);
				}
			}
		}
	}
	// camera wall
	translate([-(box_sz_x-wall)/2+cam_wall_offset, 0, 0]) {
		rotate([0, -cam_wall_angle, 0]) {
			difference() {
//				cube([wall, box_sz_y, cam_wall_sz_z], center=true);
//				cube([wall, cam_window_sz_y, cam_window_sz_z], center=true);
			}
		}
	}
}

module top_lid() {
	// top
	translate([-(middle_wall_offset-wall-gap)/2, 0, (box_sz_z-wall)/2]) {
		cube([box_sz_x-middle_wall_offset+wall+gap, box_sz_y, wall], center=true);
	}
  ear_holes(true);
}

module slot_insert() {
	difference() {
		cube([wall, box_sz_y-2*wall-gap, box_sz_z-2*wall-gap], center=true);
		cube([wall, slot_sz_y, slot_sz_z], center=true);
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
    translate([-(box_sz_x-wall)/2+cam_wall_offset-2*wall-cam_pcb_sz_x-7, 0, 0]) {
      difference() {
        rotate([0, -cam_wall_angle, 0]) {
          difference() {
            cube([wall, box_sz_y-2*wall, cam_wall_sz_z-2*wall], center=true);
            translate([0, 0, -cam_wall_sz_z/2+2*wall]) {
              cube([wall, cam_wire_sz_y, cam_wire_sz_z], center=true);
            }
          }
        }
        translate([cam_pcb_sz_x, 0, 0]) {
          rotate([0, -cam_wall_angle, 0]) {
            translate([0, 0, cam_bottom_reserve+2*wall-(cam_wall_sz_z-2*wall-cam_pcb_sz_z)/2]) {
              cam_holes(d=cam_hole_d+gap, h=20);
              %cam_pcb_sub();
            }
          }
        }
      }
      for(ysgn=[-1, 1]) {
        translate([0, ysgn*(box_sz_y-3*wall-gap)/2, 0.5]) {
          difference() {
            cube([cam_wall_sz_x, wall, box_sz_z-2*wall], center=true);
            translate([cam_wall_sz_x/2, 0, (box_sz_z-2*wall)/2]) {
              rotate([0, 45, 0]) {
                cube([cam_wall_sz_z, wall, sqrt(pow(cam_wall_sz_x, 2)-pow(cam_wall_sz_z/2, 2))*2-wall/2], center=true);
              }
            }
          }
        }
      }
    }
    cam_holder_holes(d=hole_d);
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

module grate_holder_holes(d) {
  translate([-(box_sz_x-wall)/2+cam_wall_offset, 0, 0]) {
    translate([5, 0, 5]) {
      rotate([90, 0, 0]) {
        cylinder(d=d, h=2*box_sz_y, center=true);
      }
    }
  }
}

module grate_holder() {
  difference() {
    translate([-(box_sz_x-wall)/2+cam_wall_offset, 0, 0]) {
      rotate([0, -cam_wall_angle, 0]) {
        difference() {
          cube([wall, box_sz_y-2*wall, cam_wall_sz_z-2*wall], center=true);
          cube([wall, cam_window_sz_y, cam_window_sz_z], center=true);
        }
      }
      for(ysgn=[-1, 1]) {
        translate([0, ysgn*(box_sz_y-3*wall-gap)/2, 0.5]) {
          difference() {
            cube([cam_wall_sz_x, wall, box_sz_z-2*wall], center=true);
            translate([-cam_wall_sz_x/2, 0, -(box_sz_z-2*wall)/2]) {
              rotate([0, 45, 0]) {
                cube([cam_wall_sz_z, wall, sqrt(pow(cam_wall_sz_x, 2)-pow(cam_wall_sz_z/2, 2))*2-wall/2], center=true);
              }
            }
          }
        }
      }
    }
    grate_holder_holes(d=hole_d);
  }
}

translate([0, 0, 5]) {
  top_lid();
}
sp_base();
translate([middle_wall_offset, 0, 0]) {
	//slot_insert();
}
translate([0, 0, 0]) {
  cam_holder();
}
grate_holder();

