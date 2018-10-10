$fn = 40;
wall = 2;
pcb_hole_width = 21;
pcb_hole_length = 45;
stand_length = 5;
stand_width = 5;
box_main_height = 13;
stand_height = 3;

module stand() {
  cube([stand_length, stand_width, stand_height]);
}

module hole() {
  translate([2.5, 2.5, -10]) {
    cylinder(r=1.1, h=20);
  }
}

module box() {
  difference() {
  
    union() {
      difference() {
        translate([-2, -2, -2]) {
          cube([pcb_hole_length+wall*2,
            pcb_hole_width+wall*2, box_main_height]);
          translate([0, -1, box_main_height]) {
            difference() {
              // top
              cube([pcb_hole_length+wall*2,
                pcb_hole_width+wall*2+2, 4]);
              // top hole
              translate([0, 2, 1]) {
                cube([pcb_hole_length+wall*2,
                  pcb_hole_width+wall*2-2, 2]);
                translate([0, 1, 2]) {
                  cube([pcb_hole_length+wall*2,
                    pcb_hole_width, 2]);
                }
              }
            }
          }
        }
        cube([pcb_hole_length, pcb_hole_width, 20]);
        // side hole
        translate([pcb_hole_length, 4.5, 0]) {
          cube([20, 13, 11]);
        }
        // hole for lid
        translate([-5, pcb_hole_width/2,
            box_main_height-3.5]) {
          rotate([0, 90, 0]) {
            cylinder(r=1.1, h=10); 
          }
        }
        // hole for reset
        translate([pcb_hole_length-8-2.5,
            pcb_hole_width-3, stand_height+0.5]) {
          cube([5, 10, 3]);
        }
      }
      
      stand();
      translate([0, pcb_hole_width-stand_width, 0]) {
        stand();
      }
      translate([pcb_hole_length-stand_length, 0, 0]) {
        stand();
      }
      translate([pcb_hole_length-stand_length,
          pcb_hole_width-stand_width, 0]) {
        stand();
      }
    }
  
    translate([pcb_hole_length-stand_length, 0, 0]) {
      hole();
    }
    translate([pcb_hole_length-stand_length, 16, 0]) {
      hole();
    }
    // camera hole
    translate([13, pcb_hole_width/2, -5]) {
      cylinder(r=4, h=10);
    }
    
    
  }
}

module close_lid() {
color("blue")
  translate([-3, -2, -2]) {
    difference() {
      translate([-1, -1, box_main_height]) {
        translate([wall, 2, 1]) {
          cube([pcb_hole_length+wall*2+2,
            pcb_hole_width+wall*2-2-0.4, 2-0.4]);
          translate([-wall, 0, -10+4]) {
            cube([wall, pcb_hole_width+wall*2-2-0.4, 10]);
          }
        }
      }
      // hole to screw
      translate([-10, pcb_hole_width/2+2,
          box_main_height-3.5+2]) {
        rotate([0, 90, 0]) {
          cylinder(r=1.2, h=50); 
        }
      }
    }
  }
  connector();
}

module connector() {
difference() {
  translate([4, -5, 3.2]) {
    difference() {
      rotate([90, 0, 90]) {
        import("tripod_adapter.stl");
      }
      translate([-50, -50, 0]) {
        cube([100, 100, 10]);
      }
    }
  }
}

}

box();
close_lid();
