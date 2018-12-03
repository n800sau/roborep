use <MCAD/boxes.scad>

$fn=50;

height = 25;

glass_x = 102;
glass_y = 82;
glass_h = 2.7;
wall = 3;

wet_x = glass_x + 25;
wet_y = glass_y + 10;

comp_site_top_h = wall+glass_h+15;
comb_site_tooth_h = 5;
wire_holder_z = 5;

wire_d = 2;

istep = 10;
comp_site_width = 6;
comp_site_count = 9;

banana_d = 5+0.5;

hole_d = 3.2;

font = "Liberation Sans";

comb_thick = 4;

module comb() {
	// handle
	cube([comb_thick, wet_y, 10], center=true);
}

module banana_hole(socket=false) {
  translate([0, glass_y/2, height/2-10]) {
    rotate([90, 0, 0]) {
      if(socket) {
        translate([-1, 5, -10]) {
          cylinder(d=banana_d, h=50, center=true);
        }
      } else {
        translate([-2, 1, -22]) {
          roundedBox([10, 16, 30], 5, true);
          translate([0, 4, 0]) {
            cylinder(d=hole_d, h=50, center=true);
          }
          // for wire
          translate([7, 4, 0]) {
            cylinder(d=2.5, h=50, center=true);
          }
        }
      }
    }
  }
}

module wire_holder() {
  d_pad = 2;
  d = wire_d + d_pad*2;
  w = 2;
  translate([-d/4, 0, -d/4*3]) {
    union() {
      difference() {
        rotate([90, 0, 0]) {
          difference() {
            cylinder(d=d, h=w, center=true);
            cylinder(d=wire_d, h=w, center=true);
          }
        }
        translate([0, 0, -d/2]) {
          cube([d, w*2, d], center=true);
        }
        translate([-d/2, 0, 0]) {
          cube([d, w*2, d], center=true);
        }
      }
      translate([-d/4, 0, wire_d]) {
        cube([d/4*3, w, d_pad], center=true);
      }
      translate([wire_d, 0, -d/4]) {
        cube([d_pad, w, d/4*3], center=true);
      }
    }
  }
//  hh = 3.5;
//  hw = 6;
//  translate([2, 0, wall+glass_h-height/2+hh/2]) {
//    cube([3, 2, hh], center=true);
//    translate([1.5-hw/2, 0, hh/2]) {
//      cube([hw, 2, 2], center=true);
//    }
//  }
}

module chamber() {
  difference() {
//  union() {
    // walls
    cube([wet_x+2*wall, wet_y+2*wall, height], center=true);
    translate([0, 0, 0]) {
      // attach zone
      translate([0, 0, comp_site_top_h]) {
        cube([glass_x, wet_y, height], center=true);
        // comb zone
        comb_start = -comp_site_count/2;
        comb_end = comp_site_count/2;
        for(i=[comb_start:comb_end]) {
          translate([i*istep, 0, 0]) {
            translate([0, 0, -comb_site_tooth_h]) {
              cube([comp_site_width, wet_y, height], center=true);
            }
            // get low to the glass level for rubber
            translate([0, 0, -comp_site_top_h+wall+glass_h]) {
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
        cube([wet_x, glass_y, height], center=true);
      }
      // extraction bigger
      translate([0, 0, 3]) {
        cube([glass_x, glass_y, height], center=true);
      }
      translate([0, 0, 0]) {
        // extraction smaller with pads
        cube([glass_x-5, glass_y-7, height], center=true);
      }
      // plus
      translate([glass_x/2+5, 0, 0]) {
//!difference() {
//!union() {
//  translate([-10, 76, -6]) {
//  cube([17, 2, 20]);
//  }
        banana_hole();
//}
      }
      // minus
      translate([-glass_x/2-5, 0, 0]) {
        banana_hole(socket=true);
      }      
    }
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
  translate([0, 0, 0]) {
    for(i=[-1.5:1.5]) {
      translate([-glass_x/2-9, i*(glass_y/4.5), 0]) {
        wire_holder();
      }
      translate([glass_x/2+9, i*(glass_y/4.5), 0]) {
          rotate([0, 0, 180]) {
            wire_holder();
          }
      }
    }
  }
}

chamber();
translate([0, 0, 25]) {
	comb();
}


