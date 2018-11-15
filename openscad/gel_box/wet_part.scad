height = 35;
glass_x = 100;
glass_y = 80;
glass_h = 2.7;
wall = 3;

istep = 10;
comb_width = 6;
comb_places = glass_x/istep-1;

banana_d = 6;

hole_d = 2.5;

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
    rotate([0, 50, 0]) {
      cube([10, 2, 3], center=true);
    }
}

module chamber() {
  difference() {
//  union() {
    // walls
    cube([glass_x+25+2*wall, glass_y+10+2*wall, height], center=true);
    translate([0, 0, 0]) {
      // attach zone
      translate([0, 0, height-10]) {
        cube([glass_x-10, glass_y+10, height], center=true);
      }
      // comb zone
      for(i=[-comb_places/2:comb_places/2]) {
        translate([i*istep, 0, height-15]) {
          cube([comb_width, glass_y+10, height], center=true);
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
  }
  for(i=[-2:2]) {
    translate([-glass_x/2-7, i*(glass_y/4.5), -11]) {
      wire_holder();
    }
    translate([glass_x/2+7, i*(glass_y/4.5), -11]) {
      rotate([180, 0, 0]) {
        wire_holder();
      }
    }
  }
}

chamber();

