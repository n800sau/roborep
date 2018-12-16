use <MCAD/boxes.scad>

$fn=50;

height = 20;

glass_x = 102;
glass_y = 82;
glass_h = 2.7;
wall = 3;

pad_y = 2;
wet_x = glass_x + 25;
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
electrode_plate_y = wet_y;
electrode_plate_bottom = 5;

font = "Liberation Sans";

module chamber() {
  difference() {
//  union() {
    // walls
    cube([wet_x+2*wall, wet_y+2*wall, height], center=true);
    translate([0, 0, 0]) {
      // attach zone
      translate([0, 0, comb_top_h]) {
        cube([glass_x, wet_y, height], center=true);
        // comb zone
        comb_start = -comb_places/2;
        comb_end = comb_places/2;
        for(i=[comb_start:comb_end]) {
          translate([i*istep, 0, 0]) {
            translate([0, 0, -comb_tooth_h]) {
              cube([comb_width, wet_y, height], center=true);
            }
            // get low to the glass level for rubber
            translate([0, 0, -comb_top_h+wall+glass_h]) {
              if(i == comb_start) {
                cube([5, glass_y+5, height], center=true);
              } else if(i == comb_end) {
                cube([5, glass_y+5, height], center=true);
              }
            }
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
    // bottom holes
    for(x=[-glass_x/2-5, glass_x/2+5]) {
      for(y=[-glass_y/2-4, glass_y/2+4]) {
        translate([x, y, -height/2-1]) {
          cylinder(d=3.2, h=12);
        }
      }
    }
    translate([wet_x/2+wall, wet_y/2-20, 0]) {
      rotate([90, 0, 90]) {
        linear_extrude(height = 2) {
          text("PLUS", font = font, size = 5, direction = "ltr", spacing = 1 );
        }
      }
    }
    translate([-wet_x/2-wall, wet_y/2, 0]) {
      rotate([90, 0, 270]) {
        linear_extrude(height = wall-2) {
          text("MINUS", font = font, size = 5, direction = "ltr", spacing = 1 );
        }
      }
    }
  }
}

module wire_set_plus() {
	holder_outer_d = 4;
	holder_inner_d = 2;
	holder_h_offset = (holder_outer_d-height)/2;
	border_x = wall + 3;
	border_y = wall + 3;
	difference() {
		union() {
			cube([wall, electrode_plate_y-pad*2, height], center=true);
      // bottom cylinder to hold wire
			translate([(holder_outer_d-wall)/2, 0, holder_h_offset]) {
				rotate([90, 0, 0]) {
					cylinder(d=holder_outer_d, h=wet_y, center=true);
				}
			}
      // socket side pole
			translate([0, (electrode_plate_y-border_y)/2-pad*2, 0]) {
				difference() {
					cube([border_x, border_y, height], center=true);
// to test vertical groove
					translate([0, border_y/2, 0]) {
						cylinder(d=2, h=height, center=true);
					}
				}
			}
      // non-socket side pole
			translate([0, (border_y-electrode_plate_y)/2+pad*2, 0]) {
				cube([border_x, border_y, height], center=true);
			}
		}
    // horizontal groove
		translate([(holder_outer_d-wall)/2, 0, holder_h_offset]) {
			rotate([90, 0, 0]) {
				cylinder(d=holder_inner_d, h=wet_y+10, center=true);
			}
		}
    // remove bottom part
    translate([0, 0, holder_h_offset]) {
      cube([20, wet_y-10, 20], center=true);
    }
		// groove
		translate([0, wet_y/2, 0]) {
//			cylinder(d=2, h=height, center=true);
		}
	}
}


chamber();
color("blue")
translate([wet_x/2-3, 0, 10]) {
  wire_set_plus();
}

