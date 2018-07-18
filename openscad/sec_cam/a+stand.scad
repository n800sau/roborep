stlfile = "Raspberry_Pi_A+_holder.stl";

$fn = 50;
cover_thickness = 3;
tube_thickness = 2;
pcb_disk_r = 53.5/2;
cover_disk_r = 54.5/2;
cover_r = pcb_disk_r+tube_thickness;

module stick_with_holes(extra_len=0) {
  clen = 5;
  cwidth = 8;
  translate([-cwidth/2, -pcb_disk_r-clen-extra_len, 0]) {
   difference() {
      union() {
        cube([cwidth, (pcb_disk_r +extra_len + clen) * 2, cover_thickness]);
        translate([cwidth/2, 0,  0]) {
          cylinder(r=cwidth/2, h=cover_thickness);
        }
        translate([cwidth/2, (pcb_disk_r+extra_len+clen)*2,  0]) {
          cylinder(r=cwidth/2, h=cover_thickness);
        }
      }
      translate([cwidth/2, 0,  0]) {
        cylinder(r=1.8, h=cover_thickness);
      }
      translate([cwidth/2, (pcb_disk_r+extra_len+clen)*2,  0]) {
        cylinder(r=1.8, h=cover_thickness);
      }
    }
  }
}


module morphed() {
  hull() {
    rotate(45) {
      translate([-31.5, -30, 17]) {
        cube([cover_r*2+6, cover_r*2, cover_thickness]);
      }
    }
    cylinder(r=cover_r, h=cover_thickness);
  }
}

module top_cover() {
  translate([0, 0, 0]) {
    rotate(45) {
      difference() {
        union() {
          cylinder(h=cover_thickness*4, r=cover_r);
          stick_with_holes();
          rotate(90) {
            stick_with_holes();
          }
          translate([0, 0, 10]) {
            difference() {
              union() {
                morphed();
                rotate(45) {
                  translate([15, 0, 17]) {
                    stick_with_holes(0);
                  }
                  translate([-15, 0, 17]) {
                    stick_with_holes(0);
                  }
                }
                rotate(135) {
                  translate([0, 0, 17]) {
                    stick_with_holes(5);
                  }
                }
              }
              scale([0.9, 0.9, 1]) {
                morphed();
              }
            }
          }
        }
        cylinder(h=cover_thickness*4, r=pcb_disk_r-1);
      }
    }
  }
}

module rpi_plate() {
  
  translate([0, 65, 0]) {
    rotate([90, 90, 0]) {
      top_cover();
    }
  }

  translate([5, 0, 0]) {
    difference() {
      union() {
        // one board
  //      translate([16, 30, -1.5]) {
        translate([17, 25, 0]) {
          cube([10, 25, 3]);
        }
        // another board
        translate([-37, 25, 0]) {
          cube([10, 25, 3]);
        }
      }
    }
    difference() {
      translate([50, -65, 0]) {
        import(stlfile, convexity=10);
      }
      translate([-42.1, 0, 0]) {
        cube([10, 100, 50], center=true);
      }
      translate([32, 0, 0]) {
        cube([10, 100, 50], center=true);
      }
      translate([-5, -8, -20]) {
        cube([50, 45, 50], center=true);
      }
    }
  }
}

difference() {
  rpi_plate();
  translate([29, -50, -15]) {
    cube([5, 85, 30]);
  }
  translate([-34, -50, -15]) {
    cube([5, 85, 30]);
  }
  translate([-35, 23, 3]) {
    cube([70, 5, 30]);
  }
  translate([-35, -44.5, 3]) {
    cube([70, 5, 30]);
  }
}

