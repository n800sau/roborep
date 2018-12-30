use <MCAD/boxes.scad>

$fn=50;

height = 20;

glass_x = 100.8;
glass_y = 82;
glass_h = 2.7;
wall = 3;

pad_y = 2;
wet_x = glass_x + 20;
wet_y = glass_y + 10 + pad_y*2;

comb_top_h = wall+glass_h+15;
comb_tooth_h = 5;
wire_holder_z = 5;

wire_d = 2;

istep = 10;
comb_width = 6;
comb_places = 9;

pad = 0.5;
electrode_plate_x = 5;
electrode_top_x = 6;
electrode_plate_y = wet_y-pad*2;
electrode_plate_bottom = 5;

font = "Liberation Sans";

module top_hole(xpos=0) {
  hole_d = 3.1;
  hole_h = 50;
  for(y=[wet_y-wall, wall-wet_y]) {
    translate([xpos, y/2, 0]) {
      rotate([0, 0, 90]) {
        cylinder(d=hole_d, h=hole_h, center=true);
      }
    }
  }
}

module chamber() {
  difference() {
    union() {
      // walls
      cube([wet_x+2*wall, wet_y+2*wall, height], center=true);
    }
    // holes through
    for(x=[17.5, 80, 100]) {
      top_hole((x-wet_x)/2);
      top_hole((wet_x-x)/2);
    }
    translate([0, 0, 0]) {
      // attach zone
      translate([0, 0, comb_top_h]) {
        cube([glass_x, wet_y, height], center=true);
        // comb zone
        comb_start = -comb_places/2;
        comb_end = comb_places/2;
        for(i=[comb_start+1,0,comb_end-1]) {
          translate([i*istep, 0, -comb_tooth_h]) {
            cube([comb_width, wet_y, height], center=true);
          }
        }
        for(i=[-1,1]) {
          translate([i*istep*1.75, 0, -comb_tooth_h]) {
            //cube([comb_width*2.5, wet_y, height], center=true);
          }
        }
        for(i=[comb_start,comb_end]) {
            // get low to the glass level for rubber
            translate([i*istep, 0, -comb_top_h+wall+glass_h]) {
              cube([comb_width, wet_y, height], center=true);
              cube([comb_width, wet_y, height], center=true);
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

holder_outer_d = 4;
holder_inner_d = 2;
holder_h_offset = (holder_outer_d-height)/2;
border_x = wall; // limited by gel box
border_y = wall + 3;
bolt_plate_x = 10;
bolt_plate_y = wall;
bolt_plate_z = 10;
bolt_plate_case_base_y = 30+wall*2;

module wire_set() {
	difference() {
		union() {
      translate([(electrode_top_x-wall)/2, 0, (height-wall)/2]) {
        cube([electrode_top_x, electrode_plate_y+wall, wall], center=true);
      }
			cube([wall, electrode_plate_y-pad*2, height], center=true);
			// bottom cylinder to hold wire
			translate([(holder_outer_d)/2-1, 0, c]) {
				rotate([90, 0, 0]) {
					cylinder(d=holder_outer_d, h=electrode_plate_y-pad*2, center=true);
				}
			}
			// socket side pole
			translate([border_x/2, (electrode_plate_y-border_y)/2-pad, 0]) {
				difference() {
					cube([border_x, border_y, height], center=true);
				}
			}
			// non-socket side pole
			translate([border_x/2, (border_y-electrode_plate_y)/2+pad, 0]) {
				cube([border_x, border_y, height], center=true);
			}
		}
		// horizontal wire holder
		translate([(holder_outer_d)/2-1, 0, holder_h_offset]) {
			rotate([90, 0, 0]) {
				cylinder(d=holder_inner_d, h=wet_y+10, center=true);
			}
		}
		// remove bottom part
		translate([0, 0, holder_h_offset]) {
			cube([20, glass_y, 20], center=true);
		}
		// vert groove
    for(ypos=[-wet_y/2+1, wet_y/2-1]) {
      translate([1, ypos, 0]) {
        cylinder(d=2, h=height+10, center=true);
      }
		}
    // attach holes
    for(ypos=[-electrode_plate_y/2+29, electrode_plate_y/2-29]) {
      translate([(electrode_top_x-wall)/2, ypos, 0]) {
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

chamber();
color("blue")
translate([wet_x/2-3, 0, 3]) {
  difference() {
    union() {
//      wire_set();
      translate([-5, 0, wall-pad]) {
//      wire_set_attach();
        translate([(bolt_plate_x-border_x)/2, (electrode_plate_y-bolt_plate_case_base_y)/2+wall, 24]) {
//          wire_set_cover();
          %translate([2.5, 3, -7]) {
            rotate([90, 0, 0]) {
//              cylinder(d=3, h=13.5, center=true);
            }
          }
        }
      }
    }
  }
}

