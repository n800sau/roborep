// to show stuff
show_light_enclosure = 1;
show_light_enclosure_lid = 1;
show_pcb_panel = 1;

use <ear.scad>
use <led_pcb_holder.scad>
use <pcb_led_cvf.scad>
use <wet_part_v2.scad>

$fn=50;

include <const.scad>

module pcb_panel_holes(d) {
  x_step = lenc_sz_x/4;
  for(pos=[
      [-3*x_step/2, lenc_h/4],
      [3*x_step/2, lenc_h/4],
      [-3*x_step/2, -lenc_h/4],
      [3*x_step/2, -lenc_h/4]
      ]) {
    translate([pos[0], 0, pos[1]]) {
      rotate([90, 0, 0]) {
        cylinder(d=d, h=50, center=true);
      }
    }
  }
}

module light_enclosure() {
  difference() {
    union() {
      cube([lenc_sz_x-pad*2, lenc_sz_y-pad*2, lenc_h], center=true);
      // attachment to chamber
      for(x_p=[[-1, 1, 1], [1, 1, 1], [-1, 2, 0], [1, 2, 0]]) {
        rotate([0, 0, x_p[2]*180])
        translate([x_p[0]*(hole_x_pos_list[x_p[1]]-wet_x)/2, (wet_y/2+wall)/2, (lenc_h-wall)/2]) {
          cube([10, wet_y/2+wall, wall], center=true);
        }
      }
      // holes for bottom lid
      for(pos=[
        [-1, 1, 0],
        [-1, -1, 0],
        [1, 1, 180],
        [1, -1, 180]
          ]) {
        translate([pos[0]*(lenc_sz_x/2-pad), pos[1]*(lenc_sz_y/2-10), -lenc_h/2]) {
          rotate([180, 0, pos[2]]) {
            bolt_hole_cone(wall_extra=0, height_extra=5);
          }
        }
      }
      // holes for wires
      for(x_sgn=[-1, 1]) {
        translate([x_sgn * (lenc_sz_x/2), 0, lenc_h/2-5]) {
          rotate([90, 0, 0]) {
            difference() {
              cylinder(d=5+4, h=wall, center=true);
              cylinder(d=5, h=wall, center=true);
            }
          }
        }
      }
    }
    cube([lenc_sz_x-wall*2, lenc_sz_y-wall*2, lenc_h], center=true);
    translate([0, -lenc_sz_y/2, 0]) {
      pcb_panel_holes(d=hole_d);
    }
    x_step = lenc_sz_x/4;
    for(pos=[
        [-3*x_step/2, lenc_h/4],
        [3*x_step/2, lenc_h/4],
        [-3*x_step/2, -lenc_h/4],
        [3*x_step/2, -lenc_h/4]
        ]) {
      translate([pos[0], -lenc_sz_y/2, pos[1]]) {
        rotate([90, 0, 0]) {
          cylinder(d=hole_d, h=50, center=true);
        }
      }
    }
  }
}

module light_enclosure_lid() {
  cube([lenc_sz_x-pad*2, lenc_sz_y-pad*2, wall], center=true);
      // holes for bottom lid
      for(pos=[
        [-1, 1, 0],
        [-1, -1, 0],
        [1, 1, 180],
        [1, -1, 180]
          ]) {
        translate([pos[0]*(lenc_sz_x/2-pad-0.4), pos[1]*(lenc_sz_y/2-10), -wall/2]) {
          rotate([180, 0, pos[2]]) {
            bolt_hole_cone(wall_extra=0, height_extra=0, hole_d=4);
          }
        }
      }
}

module pcb_panel() {
  difference() {
    union() {
      cube([lenc_sz_x-12, wall, lenc_h], center=true);
      %translate([7, -7, 0]) {
        //cube([power_pcb_sz_x, power_pcb_sz_y, power_pcb_h], center=true);
      }
      translate([-18, -0.3, (-lenc_h+wall)/2]) {
        rotate([90, 0, 0]) {
          pcb_led_cvf();
        }
      }
    }
    pcb_panel_holes(d=hole_through_d);
    // power hole
    translate([-28, 0, 0]) {
      rotate([90, 0, 0]) {
        cylinder(d=8.8, h=50 , center=true);
      }
    }
    // wire hole
    for(sgn=[-1,1]) {
      translate([-21, 0, sgn*9]) {
        rotate([90, 0, 0]) {
          cylinder(d=5, h=50 , center=true);
        }
      }
    }
    translate([35, 0, 0]) {
      rotate([90, 0, 0]) {
        cylinder(d=5, h=50 , center=true);
      }
    }
  }
}

//------------------------------------------------------
// main show
//------------------------------------------------------

if(show_pcb_panel) {
  color("green") {
    translate([0, (-lenc_sz_y-wall)/2-stand_sz, -(lenc_h+chamber_h)/2]) {
      pcb_panel();
    }
  }
}

if(show_light_enclosure) {
  color("orange")
    difference() {
      translate([0, 0, -(lenc_h+chamber_h)/2]) {
        light_enclosure();
      }
      holes_through(d=hole_through_d);
    }
    %translate([25, lenc_sz_y/2+10, -lenc_h/2-11]) {
      rotate([90, 30, 0]) {
        led_pcb_holder();
      }
    }
}

if(show_light_enclosure_lid) {
  color("brown")
    difference() {
      translate([0, 0, -(lenc_h+(chamber_h+wall)/2+1)]) {
        light_enclosure_lid();
      }
    }
}

