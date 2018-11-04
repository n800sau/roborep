$fn = 50;

pcb_l = 67;
pcb_w = 39;
pcb_h = 2;
pcb_elems_h = 13;
pcb_under_h = 3;

left_pad = 15;
pad = 1;
lid_pad = 0.5;

holes_dist_l = 61;
holes_dist_w = 33;
hole_off = 3.5;
hole_d = 3.2;
hole_pass_d = 4;
hole_stand_d = hole_d+4;

wall = 2;
wall_h = wall + pcb_under_h + pcb_h + pcb_elems_h;

font = "Liberation Sans";

module lid() {
  translate([-left_pad-pcb_l/2-wall-lid_pad, -pcb_w/2-wall-pad-lid_pad, wall_h]) {
    difference() {
      translate([-wall, -wall, -wall]) {
        cube([left_pad+pcb_l+wall*4+pad+lid_pad*2, pcb_w+wall*4+pad*2+lid_pad*2, wall*2+1]);
      }
      translate([0, 0, -wall_h]) {
        cube([left_pad+pcb_l+wall*2+pad+lid_pad*2, pcb_w+wall*2+pad*2+lid_pad*2, wall_h]);
      }
      // hole for out
      translate([left_pad+pcb_l-lid_pad-wall*2-1, wall+lid_pad+15, 0]) {
        cube([8, 10, wall*2]);
      }
      // hole for voltage control
      translate([44, pcb_w+pad, 0]) {
        cube([5, 5, wall*2]);
      }
      // hole for numbers and buttons
      translate([26, -6, -wall]) {
        cube([51, 14+6, wall*4]);
      }
    translate([wall, wall*2+pad+lid_pad+pcb_w/2, wall]) {
      linear_extrude(height = 2) {
        text("DC STEP DOWN", font = font, size = 7, direction = "ltr", spacing = 1 );
      }
    }
    }
  }
  translate([-left_pad-pcb_l/2, -pcb_w/2-pad, wall-lid_pad]) {
    //for holes for roof
    roof_holes(with_ears=true);
  }
}

module a_hole(with_ears=false) {
  if(with_ears) {
    difference() {
//    union() {
      translate([0, 0, 15]) {
        cube([hole_pass_d+4, hole_pass_d+4, wall], center=true);
      }
      cylinder(d=hole_pass_d, h=30);
    }
  } else {
    cylinder(d=hole_d, h=30);
  }
}

module roof_holes(with_ears=false) {
          // hole 1
          translate([25, pcb_w+20+lid_pad, wall_h-6]) {
            rotate([90, 0, 0]) {
              a_hole(with_ears);
            }
          }
          // hole 2
          translate([pcb_l+pad*2+wall/2+wall-lid_pad, pcb_w/2+11, wall_h-6]) {
            rotate([0, 90, 0]) {
              a_hole(with_ears);
            }
          }
          // hole 3
          translate([pcb_l+pad*2+wall/2+wall-lid_pad, pad+pcb_w/2-11, wall_h-6]) {
            rotate([0, 90, 0]) {
              a_hole(with_ears);
            }
          }
          // hole 4
          translate([10, (pcb_w-15)/2-lid_pad, wall_h-6]) {
            rotate([90, 0, 0]) {
              a_hole(with_ears);
            }
          }
          // hole 5
          translate([-(pcb_l-29)/2+lid_pad, pcb_w/2-10, wall_h-6]) {
            rotate([0, 90, 0]) {
              a_hole(with_ears);
            }
          }
}

module box() {
  difference() {
    union() {
      difference() {
  //    union() {
        translate([-left_pad-pcb_l/2-wall, -pcb_w/2-wall-pad, 0]) {
          cube([left_pad+pcb_l+wall*2+pad, pcb_w+wall*2+pad*2, wall_h]);
        }
        translate([-left_pad-pcb_l/2, -pcb_w/2-pad, wall]) {
          cube([left_pad+pcb_l+pad, pcb_w+2*pad, wall_h]);
          // hole for buttons and numbers
          translate([left_pad+8.5, -pcb_w/2+wall, pcb_under_h+pcb_h+(pcb_elems_h+wall)/2]) {
            cube([50, 20, (pcb_elems_h+wall)/2]);
          }
          // hole for out
          translate([left_pad+pcb_l, 14, pcb_under_h+pcb_h]) {
            cube([50, 12, pcb_elems_h+wall]);
          }
          // hole for power
          translate([7, pcb_w+15, wall_h/2]) {
            rotate([90, 0, 0]) {
              cylinder(d=9, h=30);
            }
          }
          //holes for roof
          roof_holes();
        }
      }
      for(x=[0:1]) {
        for(y=[0:1]) {
          translate([x*holes_dist_l-holes_dist_l/2,
              y*holes_dist_w-holes_dist_w/2, wall]) {
            cylinder(d=hole_stand_d, h=pcb_under_h);
          }
        }
      }
    }
    for(x=[0:1]) {
      for(y=[0:1]) {
        translate([x*holes_dist_l-holes_dist_l/2,
            y*holes_dist_w-holes_dist_w/2, -25]) {
          cylinder(d=hole_d, h=50);
        }
      }
    }
  }
}

translate([0, 0, 0]) {
  lid();
}
//box();
