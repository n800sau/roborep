//cube([122, 82, 55]);

x_size= 135;
y_size = 93;
z_size = 50;

bottom_height = 1;

border_height = bottom_height + 10;
border = 2;
wall = 2;

hole_h = 30;


module hole(stand=false, angle=0) {
  if(stand) {
    //cylinder(d=6, h=z_size);
    rotate([0, 0, angle]) {
      translate([0, -4, z_size/2]) {
        cube([8.5, 16, z_size], center=true);
      }
    }
  } else {
    translate([0, 0, z_size-hole_h]) {
      cylinder(d=3.2, h=hole_h);
    }
  }
}

module holes(stand=false) {
  translate([7, 7, 0]) {
    translate([5, 30, 0]) {
      hole(stand=stand, angle=-90);
    }
    translate([100, 5, 0]) {
      hole(stand=stand);
    }
    translate([117, 57, 0]) {
      hole(stand=stand, angle=135);
    }
    translate([68, 74, 0]) {
      hole(stand=stand, angle=180);
    }
    translate([76.5, 74, 0]) {
      hole(stand=stand, angle=180);
    }
  }
}

module pcb_hole(stand=false) {
  if(stand) {
    cylinder(d=6, h=6);
  } else {
    translate([0, 0, -hole_h/2]) {
      cylinder(d=2, h=hole_h);
    }
  }
}

module pcb_holes(stand=false) {
        translate([border+15, 10, bottom + 3]) {
          //cube([50, 70, 2]);
          translate([3, 3, 0]) {
            pcb_hole(stand=stand);
            translate([0, 64, 0]) {
              pcb_hole(stand=stand);
            }
            translate([44, 64, 0]) {
              pcb_hole(stand=stand);
            }
            translate([44, 0, 0]) {
              pcb_hole(stand=stand);
            }
          }
        }
}

module stepstand() {
  mirror() {
    difference() {
      union() {
        difference() {
  //      union() {
          union() {
            cube([x_size, y_size, border_height]);
            translate([0, 41, border_height]) {
              difference() {
//              union() {
                cube([wall, 45, 16]);
                // power hole
                translate([hole_h/2, 8, 8]) {
                  rotate([0, 90, 0]) {
                    cylinder(d=9, h=hole_h, center=true);
                  }
                }
                // power switch
                translate([0, 30, 10]) {
                  rotate([0, 0, 90]) {
                    switch_holes();
                  }
                }
              }
            }
          }
          translate([border, border, bottom_height]) {
            cube([x_size-border*2, y_size-border*2, z_size]);
          }
        }
        holes(true);
        // pcb
        pcb_holes(stand=true);
      }
      pcb_holes();
      holes();
    }
  }
}

module switch_holes() {
  cube([9, 50, 4], center=true);
  translate([9.5, 0, 0]) {
    rotate([90, 0, 0]) {
      cylinder(d=2.5, h = 50, center=true);
    }
  }
  translate([-9.5, 0, 0]) {
    rotate([90, 0, 0]) {
      cylinder(d=2.5, h = 50, center=true);
    }
  }
}

stepstand();
