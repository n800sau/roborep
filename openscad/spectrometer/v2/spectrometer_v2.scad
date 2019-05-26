$fn = 36;

wall = 2;

grating_sz = 53;
grating_sz_z = 1.5;

grating_window_sz = 40;

tube_sz = 60;
tube_len = 120;

module grating_insert() {
  difference() {
    union() {
      translate([0, 0, wall/2+grating_sz_z+wall]) {
        // top
        cube([grating_sz+2*wall, grating_sz+2*wall, wall], center=true);
      }
      translate([0, 0, wall+grating_sz_z/2]) {
        // middle
        cube([grating_sz+2*wall, grating_sz+2*wall, grating_sz_z], center=true);
      }
      translate([0, 0, wall/2]) {
        // bottom
        cube([grating_sz+2*wall, grating_sz+2*wall, wall], center=true);
        translate([0, -grating_sz/2, 8]) {
          cube([grating_sz-14, wall*2, 15], center=true);
        }
      }
    }
    cube([grating_window_sz, grating_window_sz, 50], center=true);
    translate([0, 0, wall+grating_sz_z/2]) {
      // middle
      translate([0, wall, 0]) {
        cube([grating_sz, grating_sz+wall, grating_sz_z], center=true);
      }
      cube([20, grating_sz+2*wall, grating_sz_z], center=true);
      translate([0, -grating_sz/2, 8]) {
        cube([grating_sz-22, wall*2, 4], center=true);
      }
    }
  }
}

module tube_block() {
  translate([0, 0, tube_sz/2+wall]) {
    translate([-4+wall/2, 0, -(tube_sz+wall)/2]) {
      // bottom
      cube([tube_len+wall+4, tube_sz+2*wall, wall], center=true);
    }
    for(ysgn=[-1,1]) {
      translate([0, ysgn*(tube_sz+wall)/2, 0]) {
        cube([tube_len, wall, tube_sz], center=true);
      }
    }
    translate([-tube_len/2, 0, -wall/2]) {
      difference() {
        union() {
          cube([wall, tube_sz+2*wall, tube_sz+wall], center=true);
          translate([-wall, 0, 0]) {
          }
          translate([-4-wall, 0, 0]) {
            cube([wall, tube_sz+2*wall, tube_sz+wall], center=true);
          }
          for(ysgn=[-1,1]) {
            translate([-4+wall/2, ysgn*(tube_sz+wall)/2, 0]) {
              cube([4, wall, tube_sz+wall], center=true);
            }
          }
        }
        translate([0, 0, 10]) {
          cube([30, 45, 60], center=true);
        }
      }
    }
  }
}

module camera_block() {
  translate([0, -tube_sz/2-wall, 0]) {
    cube([40, 90, wall], centre=true);
  }
  translate([0, tube_sz/2, 0]) {
    cube([wall, 90-tube_sz/2, tube_sz+wall], centre=true);
  }
}

translate([5, -10, 0]) {
  rotate([0, 0, 45]) {
    translate([10, 0, 51/2+2*wall]) {
      rotate([90, 0, 90]) {
        grating_insert();
      }
    }
  }
}

translate([-tube_len/2, 0, 0]) {
//  tube_block();
}
//camera_block();

