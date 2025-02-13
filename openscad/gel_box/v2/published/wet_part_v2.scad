// to show stuff
show_chamber = 1;
show_leg = 1;
show_comb = 1;
show_barrier = 1;
show_wireset_carbon = 1;
show_wireset_holder = 1;
show_wireset_attach = 1;
show_wireset_cover = 1;

use <ear.scad>
use <threads.scad>
include <const.scad>

$fn=50;


module top_hole(xpos=0, d) {
  hole_h = 50;
  for(y=[wet_y-wall, wall-wet_y]) {
    translate([xpos, y/2, 0]) {
      rotate([0, 0, 90]) {
        cylinder(d=d, h=hole_h, center=true);
      }
    }
  }
}

module holes_through(d) {
  // holes through
  for(x=hole_x_pos_list) {
    top_hole((x-wet_x)/2, d);
    top_hole((wet_x-x)/2, d);
  }
}

module chamber() {
  difference() {
    union() {
      // walls
      cube([wet_x+2*wall, wet_y+2*wall, height], center=true);
    }
    holes_through(hole_d);
    translate([0, 0, 0]) {
      // attach zone
      translate([0, 0, comb_place_top_h]) {
        cube([glass_x, wet_y, height], center=true);
        // comb zone
        for(i=[comb_start+1,comb_start+2,comb_start+3,0,comb_end-3,comb_end-2,comb_end-1]) {
          translate([i*istep, 0, -comb_place_h]) {
            cube([comb_place_sz_x, wet_y, height], center=true);
          }
        }
        // gaps inbetween if needed
        for(i=[-1,1]) {
          translate([i*istep*1.75, 0, -comb_place_h]) {
            //cube([comb_place_sz_x*2.5, wet_y, height], center=true);
          }
        }
        for(i=[comb_start,comb_end]) {
            // get low to the glass level for rubber
            translate([i*istep, 0, -comb_place_top_h+wall+glass_h]) {
              cube([comb_place_sz_x, wet_y, height], center=true);
              cube([comb_place_sz_x, wet_y, height], center=true);
            }
        }
      }
      // water zone
      translate([0, 0, 3+glass_h]) {
        cube([wet_x, glass_y+pad_y*2, height], center=true);
      }
      // extraction bigger
      translate([0, 0, 3]) {
        cube([glass_x, glass_y, height], center=true);
      }
      translate([0, 0, 0]) {
        // extraction smaller with pads
        cube([glass_x-5, glass_y-7, height], center=true);
      }
      // extraction for electrode
      for(x=[(wet_x-electrode_plate_x)/2, (-wet_x+electrode_plate_x)/2]) {
        translate([x, 0, glass_h]) {
          cube([electrode_plate_x+pad, electrode_plate_y, height], center=true);
        }
      }
    }
  }
}

module wire_set_carbon() {
	difference() {
		union() {
      translate([carbon_sz_x/2, 0, (-height+carbon_h)/2+1.5]) {
        cube([carbon_sz_x, carbox_sz_y, carbon_h], center=true);
        translate([(border_x-carbon_sz_x)/2, 0, -carbon_h/2]) {
          cube([border_x-pad*2, electrode_plate_y-pad*3, wall-pad], center=true);
        }
        // top plate
        translate([(-carbon_sz_x+electrode_top_x)/2, 0, (height-wall-wall)/2]) {
          //cube([electrode_top_x, 22, wall], center=true);
        }
      }
      translate([carbon_d/2, 0, (carbon_h-carbon_d)/2]) {
        rotate([90, 0, 0]) {
          difference() {
            cylinder(d=carbon_d, h=carbox_sz_y, center=true);
          }
        }
      }
		}
        translate([(carbon_d)/2, 0, (carbon_h-carbon_d)/2]) {
          rotate([90, 0, 0]) {
            difference() {
              cylinder(d=carbon_wire_hole_d, h=carbox_sz_y, center=true);
            }
          }
        }
    // attach holes
    for(ypos=[
        -electrode_plate_y/2+41,
        electrode_plate_y/2-41
      ]) {
      translate([(electrode_top_x-wall)/2, ypos, 0]) {
//        cylinder(d=2.8, h=50, center=true);
      }
    }
	}
}

module wire_set_carbon_holder() {
	difference() {
		union() {
      translate([electrode_top_x/2, 0, (height-wall)/2]) {
        cube([electrode_top_x, electrode_plate_y+wall, wall], center=true);
      }
      // sides
      for(y_sgn=[-1, 1]) {
        translate([border_x/2, y_sgn * ((electrode_plate_y-border_y)/2-pad), 0]) {
          cube([border_x, border_y, height-wall*2], center=true);
        }
      }
		}
		// horizontal wire holder
		translate([carbon_d/2, 0, (carbon_h-carbon_d)/2]) {
			rotate([90, 0, 0]) {
				cylinder(d=wire_hole_d, h=wet_y+10, center=true);
			}
		}
		// vert groove
    for(ysgn=[-1, 1]) {
      translate([border_x/2, ysgn * (wet_y/2-1), 0]) {
        cylinder(d=wire_hole_d, h=height+10, center=true);
        translate([0, ysgn * 1.5, 0]) {
          cube([2, 3, height+10], center=true);
        }
        // x dir groove
        translate([0, 0, height/2-7.5]) {
          rotate([0, 90, 0]) {
//            cylinder(d=2, h=height+10, center=true);
          }
        }
        // y dir groove
        translate([border_x/2, 0, height/2-7.5]) {
          rotate([90, 0, 0]) {
            //cylinder(d=2, h=height+10, center=true);
          }
        }
      }
		}
    // attach holes
    for(ypos=[-electrode_plate_y/2+29, electrode_plate_y/2-29]) {
      translate([electrode_top_x/2, ypos, 0]) {
        cylinder(d=3, h=50, center=true);
      }
    }
	}
}

module wire_set() {
	difference() {
		union() {
      translate([electrode_top_x/2, 0, (height-wall)/2]) {
        cube([electrode_top_x, electrode_plate_y+wall, wall], center=true);
      }
      translate([wall/2, 0, 0]) {
        cube([wall, electrode_plate_y-pad*2, height], center=true);
      }
      // sides
      for(y_sgn=[-1, 1]) {
        translate([border_x/2, y_sgn * ((electrode_plate_y-border_y)/2-pad), 0]) {
          cube([border_x, border_y, height], center=true);
        }
      }
		}
		// horizontal wire holder
		translate([border_x/2, 0, wire_holder_h_offset]) {
			rotate([90, 0, 0]) {
				cylinder(d=wire_hole_d, h=wet_y+10, center=true);
			}
		}
		// remove bottom part
		translate([0, 0, wire_holder_h_offset]) {
			cube([20, glass_y, 20], center=true);
		}
		// vert groove
    for(ysgn=[-1, 1]) {
      translate([border_x/2, ysgn * (wet_y/2-1), 0]) {
        cylinder(d=wire_hole_d, h=height+10, center=true);
        translate([0, ysgn * 1.5, 0]) {
          cube([2, 3, height+10], center=true);
        }
      }
		}
    // attach holes
    for(ypos=[-electrode_plate_y/2+29, electrode_plate_y/2-29]) {
      translate([electrode_top_x/2, ypos, 0]) {
        cylinder(d=3, h=50, center=true);
      }
    }
	}
}

module wire_set_attach() {
			// top plate for bolt
			color("green")
			translate([(bolt_plate_x-border_x+wall*2)/2,
          (electrode_plate_y-bolt_plate_case_base_y)/2, height-wall]) {
//        union() {
        difference() {
          union() {
            translate([0, 3+wall, 0]) {
              difference() {
                cube([bolt_plate_x+pad, bolt_plate_y, bolt_plate_z], center=true);
                rotate([90, 0, 0]) {
                  cylinder(d=4, h=bolt_plate_case_base_y, center=true);
                }
              }
            }
            // side
            translate([(-wall-bolt_plate_x)/2, wall, wall/2]) {
              s_h = bolt_plate_z+wall*3;
              cube([wall, bolt_plate_case_base_y, s_h+pad], center=true);
              translate([wall, 0, (s_h-wall+pad)/2]) {
                cube([wall, bolt_plate_case_base_y, wall], center=true);
              }
            }
            // bottom
            translate([0, wall, (-wall-bolt_plate_z)/2]) {
              cube([bolt_plate_x, bolt_plate_case_base_y, wall], center=true);
            }
            // out side 
            translate([(wall-bolt_plate_x)/2+pad, (bolt_plate_case_base_y+wall)/2+0.5+pad, -8]) {
              cube([bolt_plate_x, 1, 5], center=true);
            }
          }
          // attach hole
          translate([0, -11, 0]) {
            cylinder(d=4, h=50, center=true);
          }
          // vertical groove hole
          translate([-pad, (bolt_plate_case_base_y-wall)/2+pad, -8]) {
            cylinder(d=2.5, h=10, center=true);
          }
        }
			}
}

module wire_set_cover() {
//  union() {
  difference() {
    translate([wall+pad, 4+wall, 0]) {
      cube([bolt_plate_x+pad, bolt_plate_case_base_y-8-wall*2, wall], center=true);
      translate([0, (-bolt_plate_case_base_y+8+bolt_plate_y)/2, (wall-bolt_plate_z)/2]) {
        cube([bolt_plate_x+pad, bolt_plate_y, bolt_plate_z], center=true);
        loc_y = (bolt_plate_case_base_y-8-bolt_plate_y+wall)/2;
        translate([0, (wall-loc_y)/2, (-wall-bolt_plate_z)/2]) {
          cube([bolt_plate_x+pad, loc_y, wall], center=true);
        }
        // side wall
        translate([(wall+bolt_plate_x)/2, 4+wall, -wall]) {
          cube([wall, bolt_plate_case_base_y+wall, bolt_plate_z+wall*2], center=true);
        }
        // out end wall
        translate([-wall/2, bolt_plate_case_base_y/2+4+wall, -wall]) {
          cube([bolt_plate_x+wall+pad, wall, bolt_plate_z+wall*2], center=true);
        }
      }
    }
    // attachment hole
    translate([3-pad/2, -11-wall, 0]) {
      cylinder(d=4, h=50, center=true);
    }
    // site wire hole
    translate([-2, 17, -4]) {
      cube([7, 10, 2.5], center=true);
//      rotate([90, 0, 0]) {
//        cylinder(d=3, h=10, center=true);
//      }
    }
  }
}

module comb() {
  difference() {
    union() {
      // handle
      translate([(comb_sz_x-comb_handle_sz_x)/2, 0, comb_handle_h]) {
        cube([comb_handle_sz_x, wet_y+25, comb_handle_h], center=true);
      }
      // main cube with side gap along y
      cube([comb_sz_x, wet_y-pad*2, comb_h], center=true);
    }
    // remove top
    translate([0, 0, comb_place_h]) {
//      cube([comb_sz_x, wet_y-10, comb_h], center=true);
    }
    // make middle thinner
    translate([-comb_handle_sz_x, 0, 0]) {
      cube([comb_sz_x, wet_y-10, comb_h], center=true);
    }
  }
  // teeth
  translate([0, 0, (-comb_place_h-comb_tooth_h)/2]) {
    for(y=[-3:1:3]) {
      translate([(comb_sz_x-comb_well_sz_x)/2, y * 10, 0]) {
        cube([comb_well_sz_x, comb_well_sz_y, comb_tooth_h], center=true);
      }
    }
  }
}

module barrier() {
  difference() {
    union() {
      // handle
      translate([-(comb_handle_sz_x-comb_sz_x)/2, 0, (barrier_h+comb_handle_h)/2]) {
        cube([comb_handle_sz_x, wet_y+25, comb_handle_h], center=true);
      }
      // with side gap along y
      cube([barrier_sz_x, wet_y-pad*4, barrier_h], center=true);
    }
    // remove top
    translate([0, 0, 5]) {
      cube([barrier_sz_x, wet_y-18, barrier_h], center=true);
    }
    // make middle thinner
    translate([-barrier_thin_sz_x, 0, barrier_thin_h-barrier_h]) {
      cube([barrier_sz_x, wet_y, barrier_h], center=true);
    }
  }
}

module leg() {
  module leg_base(sz_y) {
    hull() {
      translate([-5, 0, 0]) {
        cube([leg_sz_x, sz_y, 1], center=true);
      }
      translate([0, 0, leg_h+leg_h_extra]) {
        cube([leg_sz_x, sz_y, 1], center=true);
      }
    }
  }
  translate([(leg_sz_x-wall)/2-pad, 0, 0]) {
    difference() {
      leg_base(wet_y+wall*2+wall*2+pad*2);
      translate([wall, 0, 0]) {
        leg_base(wet_y+wall*2+pad*2);
      }
    }
    translate([0, 0, leg_h-wall]) {
      difference() {
        cube([leg_sz_x, wet_y+wall*2+pad*2, wall], center=true);
        translate([(wet_x-glass_x)/2, 0, 0]) {
          cube([leg_sz_x, glass_y, wall], center=true);
        }
      }
    }
  }
}

//------------------------------------------------------
// main show
//------------------------------------------------------

if(show_chamber) {
  chamber();
}
if(show_leg) {
  difference() {
    union() {
      translate([0, 0, -leg_h-height/2]) {
        translate([-wet_x/2-wall, 0, 0]) {
          color("blue")
            leg();
        }
        // another leg is for preview only
        %translate([wet_x/2+wall, 0, 0]) {
          rotate([0, 0, 180]) {
            leg();
          }
        }
      }
    }
    holes_through(d=hole_through_d);
  }
}

if(show_comb)
  color("brown")
    translate([0, 0, wall+glass_h+glass_tooth_gap+1]) {
      comb();
    }

if(show_barrier)
  color("blue")
    translate([comb_start*istep, 0, (wall+glass_h)/2+pad]) {
      barrier();
    }

translate([wet_x/2-6, 0, 3]) {
  if(show_wireset_carbon) {
    color("gray") {
        wire_set_carbon();
    }
  }
  if(show_wireset_holder) {
    color("blue") {
      if(wire_set_holder_type == "carbon") {
        wire_set_carbon_holder();
      } else {
        wire_set();
      }
    }
  }
  translate([-2.5, 0, wall-pad]) {
    if(show_wireset_attach) {
      color("green")
        wire_set_attach();
    }
    translate([(bolt_plate_x-border_x)/2, (electrode_plate_y-bolt_plate_case_base_y)/2+wall, 24]) {
      if(show_wireset_cover) {
        color("brown")
          wire_set_cover();
        %translate([2.5, 3, -7]) {
          rotate([90, 0, 0]) {
            cylinder(d=3, h=13.5, center=true);
          }
        }
      }
    }
  }
}

