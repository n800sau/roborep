show_camside = false;
show_pinside = true;

use <../lib/vent.scad>
use <../lib/gopro_like_connector.scad>

$fn = 40;

// zero - left bottom

wall = 2;

corner_stand_sz_x = 2.5;
corner_stand_sz_y = corner_stand_sz_x;

pcb_sz_x = 28;
pcb_sz_y = 40.4;
pcb_sz_z = 1.5;

pcb_cam_hole_d = 8.4;
pcb_cam_hole_off_x = 14;
pcb_cam_hole_off_y = 31;

led_hole_sz_x = 4;
led_hole_sz_y = led_hole_sz_x;
led_hole_off_x = 23;
led_hole_off_y = 8;

top_y_stand_sz_z = 5;
bottom_y_stand_sz_z = 3;
top_y_wall_sz_z = top_y_stand_sz_z + pcb_sz_z;
bottom_y_wall_sz_z = bottom_y_stand_sz_z + pcb_sz_z;
bottom_part_sz_y = 18;

hole_d = 2.9;
hole_d_through = 3.6;
bolt_stand_d = hole_d_through + 1.5 * wall;

sdcard_hole_off_x = 5;
sdcard_hole_sz_x = 16;
sdcard_hole_sz_z = 4 + pcb_sz_z;

pinside_wall_sz_z = 20;

// micro usb pcb connector at zero
usb_pcb_sz_x = 14.4;
usb_pcb_sz_y = 18;
usb_pcb_sz_z = 1.8;
usb_pcb_hole_d = 3;

usb_pcb_hole_off_x = 2.5;
usb_pcb_hole_dist_x = 9;
usb_pcb_hole_off_y = 9.2;

micro_usb_sz_x = 8;
micro_usb_sz_y = 6.2;
micro_usb_sz_z = 3.3;
micro_usb_under_sz_z = 3;

module esp32cam_camside() {
  difference() { 
    union() {
      // wall
      cube([pcb_sz_x+2*wall, wall, bottom_y_wall_sz_z]);
      // bottom part 
      translate([0, 0, bottom_y_wall_sz_z]) {
        cube([pcb_sz_x+2*wall, bottom_part_sz_y+bolt_stand_d+wall, wall+top_y_wall_sz_z-bottom_y_wall_sz_z]);
      }
      // side walls
      for(x=[0, pcb_sz_x+wall]) {
        translate([x, 0, 0]) {
          cube([wall, bottom_part_sz_y+bolt_stand_d+wall, bottom_y_wall_sz_z]);
        }
      }
      translate([0, bottom_part_sz_y+bolt_stand_d, 0]) {
        translate([0, 0, top_y_wall_sz_z]) {
          cube([pcb_sz_x+2*wall, pcb_sz_y-bottom_part_sz_y+bolt_stand_d+2*wall, wall]);
        }
        // side walls
        for(x=[0, pcb_sz_x+wall]) {
          translate([x, 0, 0]) {
            cube([wall, pcb_sz_y-bottom_part_sz_y+bolt_stand_d+2*wall, top_y_wall_sz_z]);
          }
        }
      }
      // top y wall
      translate([0, pcb_sz_y+2*bolt_stand_d+wall, 0]) {
        cube([pcb_sz_x+2*wall, wall, top_y_wall_sz_z]);
      }
      // pcb stands
      for(x=[wall, wall+pcb_sz_x-corner_stand_sz_x]) {
        translate([x, wall+bolt_stand_d, 0]) {
          translate([0, 0, bottom_y_wall_sz_z-bottom_y_stand_sz_z]) {
            cube([corner_stand_sz_x, corner_stand_sz_y, bottom_y_wall_sz_z]);
          }
          translate([0, pcb_sz_y-corner_stand_sz_y, top_y_wall_sz_z-top_y_stand_sz_z]) {
            cube([corner_stand_sz_x, corner_stand_sz_y, top_y_wall_sz_z]);
          }
        }
      }
      // bolt stands
      for(x=[wall+bolt_stand_d/2, wall+pcb_sz_x-bolt_stand_d/2]) {
        for(y=[wall+bolt_stand_d/2, wall+bolt_stand_d+pcb_sz_y+bolt_stand_d/2]) {
          translate([x, wall+bolt_stand_d/2, 0]) {
            cylinder(d=bolt_stand_d, h=bottom_y_wall_sz_z);
            translate([0, pcb_sz_y+bolt_stand_d, 0]) {
              cylinder(d=bolt_stand_d, h=top_y_wall_sz_z);
            }
          }
        }
      }
    }
    // led hole
    translate([led_hole_off_x+wall, led_hole_off_y+wall+bolt_stand_d, top_y_stand_sz_z-1]) {
      hull() {
        cube([led_hole_sz_x, led_hole_sz_y, 1]);
        translate([-2, -2, top_y_stand_sz_z-bottom_y_stand_sz_z+2]) {
          cube([led_hole_sz_x+4, led_hole_sz_y+4, 1]);
        }
      }
    }
    // camera hole
    translate([pcb_cam_hole_off_x+wall, pcb_cam_hole_off_y+wall+bolt_stand_d, 0]) {
      cylinder(d=pcb_cam_hole_d, h=20);
    }
    // bolt stand holes
    for(x=[wall+bolt_stand_d/2, wall+pcb_sz_x-bolt_stand_d/2]) {
      for(y=[wall+bolt_stand_d/2, wall+bolt_stand_d+pcb_sz_y+bolt_stand_d/2]) {
        translate([x, wall+bolt_stand_d/2, 0]) {
          cylinder(d=hole_d_through, h=50);
          translate([0, pcb_sz_y+bolt_stand_d, 0]) {
            cylinder(d=hole_d_through, h=50);
          }
        }
      }
    }
    // sd card hole
    translate([wall+sdcard_hole_off_x, pcb_sz_y+2*bolt_stand_d+wall, 0]) {
      cube([sdcard_hole_sz_x, wall, sdcard_hole_sz_z]);
    }
  }
}

pinside_wall_sz_z = 20;

module esp32cam_pinside() {
  difference() { 
    union() {
      // bottom
      cube([pcb_sz_x+2*wall, pcb_sz_y+2*bolt_stand_d+2*wall, wall]);
      translate([0, 0, wall]) {
        // zero y wall
        cube([pcb_sz_x+2*wall, wall, pinside_wall_sz_z]);
        // side wall
        for(x=[0, pcb_sz_x+wall]) {
          translate([x, 0, 0]) {
            cube([wall, pcb_sz_y+2*bolt_stand_d+2*wall, pinside_wall_sz_z]);
          }
        }
        // end y wall
        translate([0, pcb_sz_y+2*bolt_stand_d+wall, 0]) {
          cube([pcb_sz_x+2*wall, wall, pinside_wall_sz_z]);
        }
        // pcb stands
        for(x=[wall, wall+pcb_sz_x-corner_stand_sz_x]) {
          translate([x, wall+bolt_stand_d, 0]) {
            cube([corner_stand_sz_x, corner_stand_sz_y, pinside_wall_sz_z]);
            translate([0, pcb_sz_y-corner_stand_sz_y, 0]) {
              cube([corner_stand_sz_x, corner_stand_sz_y, pinside_wall_sz_z]);
            }
          }
        }
        // bolt stands
        for(x=[wall+bolt_stand_d/2, wall+pcb_sz_x-bolt_stand_d/2]) {
          for(y=[wall+bolt_stand_d/2, wall+bolt_stand_d+pcb_sz_y+bolt_stand_d/2]) {
            translate([x, wall+bolt_stand_d/2, 0]) {
              cylinder(d=bolt_stand_d, h=pinside_wall_sz_z);
              translate([0, pcb_sz_y+bolt_stand_d, 0]) {
                cylinder(d=bolt_stand_d, h=pinside_wall_sz_z);
              }
            }
          }
        }
      }
      translate([0, pcb_sz_y+2*bolt_stand_d+2*wall-9.5, 20.4]) {
        rotate([-90, 0, 0]) {
          connector(wall=wall, sz_x=10, sz_y=0);
        }
      }
    }
    // bolt stand holes
    for(x=[wall+bolt_stand_d/2, wall+pcb_sz_x-bolt_stand_d/2]) {
      for(y=[wall+bolt_stand_d/2, wall+bolt_stand_d+pcb_sz_y+bolt_stand_d/2]) {
        translate([x, wall+bolt_stand_d/2, wall]) {
          cylinder(d=hole_d, h=50);
          translate([0, pcb_sz_y+bolt_stand_d, 0]) {
            cylinder(d=hole_d, h=50);
          }
        }
      }
    }
    // micro usb
    translate([(usb_pcb_sz_x+pcb_sz_x+2*wall)/2, wall+micro_usb_under_sz_z, 0]) {
      rotate([90, 0, 180]) {
        micro_usb_connector();
        for(x=[usb_pcb_hole_off_x, usb_pcb_hole_off_x+usb_pcb_hole_dist_x]) {
          translate([x, usb_pcb_hole_off_y, -10]) {
            cylinder(d=usb_pcb_hole_d+.6, h=20);
          }
        }
      }
    }
    // hole for wires or ventilation
    translate([6, 48, 0]) {
      rotate([90, 0, 0]) {
        vent(x_size=20, x_count=3, z_size=30, z_count=4, center=false, negate=true);
      }
    }
  }
}

module micro_usb_connector() {
  difference() {
    union() {
      cube([usb_pcb_sz_x, usb_pcb_sz_y, usb_pcb_sz_z]);
      translate([(usb_pcb_sz_x-micro_usb_sz_x)/2, 0, usb_pcb_sz_z]) {
        cube([micro_usb_sz_x, micro_usb_sz_y, micro_usb_sz_z]);
      }
    }
    for(x=[usb_pcb_hole_off_x, usb_pcb_hole_off_x+usb_pcb_hole_dist_x]) {
      translate([x, usb_pcb_hole_off_y, 0]) {
        cylinder(d=usb_pcb_hole_d, h=20);
      }
    }
  }
}


color("blue")
translate([(usb_pcb_sz_x+pcb_sz_x+2*wall)/2, wall+micro_usb_under_sz_z, 0]) {
  rotate([90, 0, 180]) {
//    micro_usb_connector();
  }
}

color("green") {
  translate([30, 51.5, 30])  {
    rotate([90, 0, -90]) {
      //import("ESP3-CAM_v4.stl");
    }
  }
}


if(show_camside) {
	translate([0, 0, 30]) {
		esp32cam_camside();
	}
}

if(show_pinside) {
	esp32cam_pinside();
}

