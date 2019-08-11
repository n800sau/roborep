$fn = 40;

use <../lib/switch.scad>

wall = 2;
gap = 1;
led_d = 26;
led_pcb_sz = 5;

cube_sz_x = 26;
cube_sz_y = 8.4;
cube_hole_x = 5;

v_slot_extra_sz_x = 20;

hole_d = 3.2;
hole_d_through = 3.6;

ear_sz_x = 6;
ear_sz_y = 6;
ear_sz_z = 4;

switch_box_sz_z = 12;

module led_ears() {
  cube([ear_sz_x, led_d+2*gap+2*wall+ear_sz_y, ear_sz_z], center=true);
  for(xpos=[-1,1]) {
    translate([0, xpos*(led_d+2*gap+2*wall+ear_sz_y)/2, 0]) {
      cylinder(d=ear_sz_x, h=ear_sz_z, center=true);
    }
  }
}

module led_ears_holes(d=hole_d) {
  for(xpos=[-1,1]) {
    translate([0, xpos*(led_d+2*gap+2*wall+ear_sz_y)/2, 0]) {
      cylinder(d=d, h=20, center=true);
    }
  }
}

module led_wheel_holder() {
  difference() {
    union() {
      cylinder(d=led_d+2*gap+2*wall, h =led_pcb_sz+wall, center=true);
      translate([0, 0, (-led_pcb_sz-wall+ear_sz_z)/2]) {
        led_ears();
      }
      translate([led_d/2+cube_sz_x/2, 0, 0]) {
        cube([cube_sz_x+wall+v_slot_extra_sz_x, cube_sz_y+2*wall, led_pcb_sz + wall], center=true);
        translate([(cube_sz_x+v_slot_extra_sz_x/2)/2, 0, (led_pcb_sz + wall)/2]) {
          rotate([90, 0, 0]) {
            cylinder(d=hole_d_through, h=cube_sz_y+2*wall, center=true);
          }
        }
      }
    }
    translate([0, 0, wall]) {
      cylinder(d=led_d+2*gap, h = led_pcb_sz, center=true);
      translate([0, 0, -led_pcb_sz/2]) {
        led_ears_holes();
      }
    }
    translate([led_d/2+cube_sz_x/2, 0, wall/2]) {
      cube([cube_sz_x, cube_sz_y, led_pcb_sz], center=true);
      translate([(cube_sz_x-cube_hole_x)/2, 0, -wall]) {
        cube([cube_hole_x, cube_sz_y, led_pcb_sz], center=true);
      }
      translate([(cube_sz_x+v_slot_extra_sz_x/2)/2, 0, 0]) {
        cylinder(d=hole_d_through, h=20, center=true);
      }
    }
  }
}

module switch_box() {
  difference() {
    union() {
      cylinder(d=led_d+2*gap+2*wall, h=switch_box_sz_z+wall, center=true);
      translate([0, 0, (switch_box_sz_z+wall-ear_sz_z)/2]) {
        led_ears();
      }
    }
    translate([0, 0, wall]) {
      cylinder(d=led_d+2*gap, h=switch_box_sz_z, center=true);
    }
    translate([0, 0, (switch_box_sz_z+wall-ear_sz_z)/2]) {
      led_ears_holes(d=hole_d_through);
      translate([led_d/2+cube_sz_x/2, 0, 0]) {
        cube([cube_sz_x+wall+v_slot_extra_sz_x, 8, 5], center=true);
      }
    }
    switch_holes();
  }
}

//led_wheel_holder();
translate([0, 0, -20]) {
  switch_box();
}
