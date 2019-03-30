$fn = 50;

gap = 1;
wall = 3;
electrode_sz_z = 32;

max_sz_y = 86.5;

h=max_sz_y - 2 * wall - 2 * gap - 10;

int_d = 2;
ext_d = int_d + 4;

holder_d = ext_d + 2 * wall + gap;
holder_sz_y = 10;

module isolat(extra_d=0) {
  d = ext_d+extra_d;
  rotate([90, 0, 0]) {
    difference() {
      union() {
        cylinder(d=d, h=h, center=true);
        translate([0, -d/4, 0]) {
          cube([d, d/2, h], center=true);
        }
      }
      translate([0, 0, wall*2]) {
        cylinder(d=int_d, h=h, center=true);
      }
    }
  }
}


module hook() {
  difference() {
    translate([0, holder_sz_y-h/2-wall, 0]) {
      difference() {
        union() {
          rotate([90, 0, 0]) {
            cylinder(d=holder_d, h=holder_sz_y);
          }
          translate([0, (wall-holder_d)/2, electrode_sz_z/2]) {
            cube([4+2*wall, holder_sz_y, electrode_sz_z], center=true);
            translate([0, -15+holder_sz_y/2, (electrode_sz_z+5)/2]) {
              difference() {
                cube([4+2*wall, 30, 5], center=true);
                translate([0, -7.5, 0]) {
                  cylinder(d=3.2, h=40, center=true);
                }
              }
            }
          }
        }
        translate([0, -holder_sz_y, electrode_sz_z/2+5]) {
          cube([4, holder_sz_y, electrode_sz_z+10], center=true);
        }
      }
    }
    hull() {
      isolat(0.6);
    }
  }
}

module holder_hook() {
  difference() {
    translate([0, holder_sz_y-h/2-wall, 0]) {
      difference() {
        union() {
          rotate([90, 0, 0]) {
            cylinder(d=holder_d, h=holder_sz_y);
          }
          translate([0, (wall-holder_d)/2, electrode_sz_z/2]) {
            cube([4+2*wall, holder_sz_y, electrode_sz_z], center=true);
            translate([0, +15-holder_sz_y/2, (electrode_sz_z+5)/2]) {
              difference() {
                cube([4+2*wall, 30, 5], center=true);
              }
            }
          }
        }
      }
    }
    translate([0, -10, 0]) {
        hull() {
          isolat(0.6);
        }
    }
  }
  translate([0, -18, 22]) {
    difference() {
      cube([4+2*wall, 2*wall+5.3, 20], center=true);
      translate([0, 0, ]) {
        cube([4+2*wall, 5.3, 20], center=true);
      }
    }
  }
}

module socket() {
  difference() {
    union() {
      cube([4+2*wall, 15.5, wall], center=true);
      translate([0, -7.75+wall/2, 7.5]) {
        cube([4+2*wall, wall, 15], center=true);
      }
    }
    translate([0, 3, 0]) {
      cylinder(d=4, h=40, center=true);
    }
    translate([0, 0, 10]) {
      rotate([90, 0, 0]) {
        cylinder(d=4, h=40, center=true);
      }
    }
  }
}

module lock() {
  difference() {
    union() {
      // top
      translate([0, -4.25, 10]) {
        cube([4+2*wall, 18, wall], center=true);
      }
      cube([4+2*wall, 2*wall+3.4, 20], center=true);
    }
    translate([0, 0, -wall/2]) {
      cube([4+2*wall, 3.4, 20], center=true);
    }
    translate([0, -7, 0]) {
      cylinder(d=3.2, h=50, center=true);
    }
  }
}


//isolat();
translate([0, holder_sz_y-h+9-3, electrode_sz_z+wall/2+2+wall]) {
//  socket();
}
//hook();
translate([0, h-7, 0]) {
  holder_hook();
}
translate([0, holder_sz_y-h+16, electrode_sz_z-12]) {
//  lock();
}
