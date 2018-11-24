$fn=50;

height = 25;

glass_x = 100;
glass_y = 80;
glass_h = 2.7;
wall = 3;

comb_top_h = wall+glass_h+15;
comb_tooth_h = 5;
wire_holder_z = 5;

istep = 10;
comb_width = 6;
comb_places = glass_x/istep-1;

banana_d = 5+0.5;

hole_d = 3.2;

module banana_hole() {
  translate([0, glass_y/2, height/2-10]) {
    rotate([90, 0, 0]) {
      translate([0, 0, -27]) {
        cylinder(d=banana_d, h=50, center=true);
      }
      cylinder(d=2, h=50, center=true);
      translate([0, 7, 0]) {
        cylinder(d=hole_d, h=50, center=true);
      }
    }
  }
}

module wire_holder() {
  hh = 3;
  hw = 6;
  translate([0, 0, wall+glass_h-height/2+hh/2]) {
    cube([3, 2, hh], center=true);
    translate([1.5-hw/2, 0, hh/2]) {
      cube([hw, 2, 2], center=true);
    }
  }
}

module chamber() {
  difference() {
//  union() {
    // walls
    cube([glass_x+25+2*wall, glass_y+10+2*wall, height], center=true);
    translate([0, 0, 0]) {
      // attach zone
      translate([0, 0, comb_top_h]) {
        cube([glass_x, glass_y+10, height], center=true);
        // comb zone
        comb_start = -comb_places/2;
        comb_end = comb_places/2;
        for(i=[comb_start:comb_end]) {
          translate([i*istep, 0, 0]) {
            translate([0, 0, -comb_tooth_h]) {
              cube([comb_width, glass_y+10, height], center=true);
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
        cube([glass_x+25, glass_y, height], center=true);
      }
      // extraction bigger
      translate([0, 0, 3]) {
        cube([glass_x, glass_y, height], center=true);
      }
      translate([0, 0, 0]) {
        // extraction smaller
        cube([glass_x-5, glass_y-5, height], center=true);
      }
      translate([glass_x/2+5, 0, 0]) {
        banana_hole();
      }
      translate([-glass_x/2-5, 0, 0]) {
        banana_hole();
      }      
    }
    for(x=[-glass_x/2-5, glass_x/2+5]) {
      for(y=[-glass_y/2-4, glass_y/2+4]) {
        translate([x, y, -height/2-1]) {
          cylinder(d=3.2, h=12);
        }
      }
    }
  }
  translate([0, 0, 0]) {
    for(i=[-2:2]) {
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

