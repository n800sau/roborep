$fn = 50;

gap = 0.2;
wall = 2;

relay_sz_x = 19;
relay_sz_y = 15.3;
relay_sz_z = 15.4;

holder_h = 5;
hole_d = 3.8;
ear_sz_x = wall * 2 + hole_d;
ear_sz_y = wall * 2 + hole_d;

module bottom_part() {
  difference() {
    union() {
      translate([0, 0, (holder_h+wall)/2]) {
        cube([relay_sz_x+2*gap+2*wall, relay_sz_y+2*gap+2*wall, holder_h+wall], center=true);
      }
      translate([0, 0, wall/2]) {
        cube([ear_sz_x, relay_sz_y+2*gap+2*wall+ear_sz_y, wall], center=true);
        for(sgn=[-1,1]) {
          translate([0, sgn*(relay_sz_y+2*gap+2*wall+ear_sz_y)/2, -wall/2]) {
            cylinder(d=ear_sz_y, h=wall);
          }
        }
      }
    }
    translate([0, 0, holder_h/2+wall]) {
      cube([relay_sz_x+2*gap, relay_sz_y+2*gap, holder_h], center=true);
      cube([10, relay_sz_y+2*gap+2*wall, holder_h], center=true);
    }
    for(sgn=[-1,1]) {
      translate([0, sgn*(relay_sz_y+2*gap+2*wall+ear_sz_y)/2, -25]) {
        cylinder(d=hole_d, h=50);
      }
    }
  }
}

module top_part() {
  translate([0, 0, (relay_sz_z+gap+wall)/2+wall]) {
    difference() {
      union() {
        cube([ear_sz_x, relay_sz_y+2*gap+2*wall, relay_sz_z+wall+gap], center=true);
        translate([0, 0, -(relay_sz_z+gap)/2]) {
          difference() {
            union() {
              cube([ear_sz_x, relay_sz_y+2*gap+2*wall+ear_sz_y, wall], center=true);
              for(sgn=[-1, 1]) {
//                hull() {
//                  translate([0, sgn*(relay_sz_y+2*gap+wall)/2, 4]) {
//                    cube([ear_sz_x, wall, 8], center=true);
//                  }
                  translate([0, sgn*(relay_sz_y+2*gap+2*wall+ear_sz_y)/2, 0]) {
                    cylinder(d=ear_sz_y, h=wall, center=true);
                  }
//                }
              }
            }
            for(sgn=[-1,1]) {
              translate([0, sgn*(relay_sz_y+2*gap+2*wall+ear_sz_y)/2, -wall/2]) {
                cylinder(d=3, h=20);
              }
            }
          }
        }
      }
      translate([0, 0, -(wall+gap)/2]) {
        cube([ear_sz_x, relay_sz_y+2*gap, relay_sz_z+gap], center=true);
      }
    }
  }
}

translate([0, 0, 0.5]) {
  top_part();
}
//bottom_part();
