$fn = 50;

x_size= 135;
y_size = 93;
z_size = 55;

bottom_height = 2;

border_height = bottom_height + 5;
border = 2;
wall = border;

hole_h = 30;

top_stand_h = 5;
top_h = 2;

module hole(stand=false, angle=0, h=z_size, hole_d=3.2) {
  if(stand) {
    //cylinder(d=6, h=z_size);
    rotate([0, 0, angle]) {
      translate([0, -4, h/2]) {
        cube([8.5, 16, h], center=true);
      }
    }
  } else {
    translate([0, 0, h-hole_h]) {
      cylinder(d=hole_d, h=hole_h);
    }
  }
}

module holes(stand=false, h=z_size, hole_d=3.2) {
  translate([7, 7, 0]) {
    translate([5, 30, 0]) {
      hole(stand=stand, angle=-90, h=h, hole_d=hole_d);
    }
    translate([100, 5, 0]) {
      hole(stand=stand, h=h, hole_d=hole_d);
    }
    translate([117, 57, 0]) {
      hole(stand=stand, angle=135, h=h, hole_d=hole_d);
    }
    translate([68, 74, 0]) {
      hole(stand=stand, angle=180, h=h, hole_d=hole_d);
    }
    translate([76.5, 74, 0]) {
      // conflict with connector
      //hole(stand=stand, angle=180, h=h);
    }
  }
}

module pcb_hole(stand=false) {
  if(stand) {
    cylinder(d=6, h=6);
  } else {
    translate([0, 0, -hole_h/2]) {
      cylinder(d=2.4, h=hole_h);
    }
  }
}

module pcb_holes(stand=false) {
        translate([border+15, 10, bottom_height]) {
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
            difference() {
              cube([x_size, y_size, border_height]);
// with hole it prints 4:45 instead of 5:15 and $0.76 instead of 0.86
//              translate([25, 25, 0]) {
//                cube([x_size-50, y_size-50, border_height]);
//              }
            }
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
  cube([9, 50, 5.5], center=true);
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

module standtop() {
  big_hole_x = 59.1;
  big_hole_y = 33.5;
  mirror() {
    translate([0, 0, 40]) {
      difference() {
        union() {
          cube([x_size, y_size, top_h]);
          translate([big_hole_x, big_hole_y, 1]) {
            cylinder(d=39+4, h=3.5);
          }
        }
        translate([0, 0, -z_size/2]) {
          holes(hole_d=4);
        }
        // for bearing
        translate([big_hole_x, big_hole_y, 0]) {
          cylinder(d=32, h=top_stand_h);
          translate([0, 0, 2]) {
            cylinder(d=39, h=top_stand_h);
          }
        }
        // holes for wires
        for(pos=[[10, 10],
            [10, y_size-10],
            [x_size-10, y_size-10],
            [x_size-10, 10]]) {
          translate([pos[0], pos[1], 0]) {
            cylinder(d=10, h=top_h*2);
          }
        }
        // holes for something else
        translate([x_size - 40, 15, 0]) {
          for(pos=[[10, 10],
              [10, 30],
              [30, 30],
              [30, 10],
              [10, 50]]) {
            translate([pos[0], pos[1], 0]) {
              cylinder(d=4, h=top_h*2);
            }
          }
        }
      }
    }
  }
}


translate([0, 0, 20]) {
  standtop();
}
//stepstand();
