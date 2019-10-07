use <MCAD/boxes.scad>
use <../lib/gopro_like_connector.scad>

$fn = 40;

wall = 2;

ttf_hole_sz_x = 26;
ttf_hole_sz_y = 26;
ttf_hole_shift_x = 3.8;

ttf_stand_d = 4;
ttf_stand_h = 3;
ttf_stand_hole_d = 1.8;

ttf_pcb_sz_x = 39.8;
ttf_pcb_sz_y = 28.8;
ttf_pcb_sz_z = 1.6;

border_sz_z = ttf_pcb_sz_z+ttf_stand_h;

stand_dist_x = 34.2;
stand_dist_y = 23.2;

hole_d = 2.9;
hole_d_through = 3.6;

ear_hole_d = hole_d_through;
ear_sz_x = 2;
ear_sz_y = ear_hole_d + 2 * wall;
ear_sz_z = wall+border_sz_z+5;

pins_hole_sz_x = 5;
pins_hole_sz_y = 19;


border_butt_sz_z = ear_sz_z-wall-border_sz_z;

module ears(hole_mode=false) {
  // ears
  translate([0, 0, ear_sz_z/2]) {
    for(x=[-1,1]) {
      translate([x*(ttf_pcb_sz_x+ear_sz_x+2*wall)/2, 0, 0]) {
        if(!hole_mode) {
          translate([0, 0, -ear_sz_y/4]) {
            cube([ear_sz_x, ear_sz_y, ear_sz_z+wall-ear_sz_y/2], center=true);
          }
        }
        translate([0, 0, (ear_sz_z-ear_sz_y+wall)/2]) {
          rotate([0, 90, 0]) {
            cylinder(d=hole_mode ? ear_hole_d : ear_sz_y, h=ear_sz_x+(hole_mode ? 1 : 0), center=true);
          }
        }
      }
    }
  }
}

module ttf_face_part() {
  difference() {
    union() {
      difference() {
        translate([0, 0, border_sz_z/2]) {
          roundedBox([ttf_pcb_sz_x+2*wall, ttf_pcb_sz_y+2*wall, wall+border_sz_z], 2, true);
        }
        translate([0, 0, (border_sz_z)/2+wall]) {
          roundedBox([ttf_pcb_sz_x, ttf_pcb_sz_y, wall+border_sz_z], 2, true);
        }
      }
      for(x=[-1,1]) {
        for(y=[-1,1]) {
          translate([x*stand_dist_x/2, y*stand_dist_y/2, (wall+ttf_stand_h)/2]) {
            cylinder(d=ttf_stand_d, h=ttf_stand_h, center=true);
          }
        }
      }
      ears();
    }
    // ttf hole
    translate([ttf_hole_shift_x/2, 0, 0]) {
      cube([ttf_hole_sz_x, ttf_hole_sz_y, wall*2], center=true);
    }
    // stand holes
    for(x=[-1,1]) {
      for(y=[-1,1]) {
        translate([x*stand_dist_x/2, y*stand_dist_y/2, (1+ttf_stand_h)/2]) {
          cylinder(d=ttf_stand_hole_d, h=ttf_stand_h+wall-1, center=true);
        }
      }
    }
    // ear holes
    ears(hole_mode=true);
  }
}

module ttf_butt_part() {
  difference() {
    translate([0, 0, border_butt_sz_z/2]) {
      roundedBox([ttf_pcb_sz_x+2*wall, ttf_pcb_sz_y+2*wall, wall+border_butt_sz_z], 2, true);
    }
    translate([0, 0, (border_butt_sz_z-wall)/2]) {
      roundedBox([ttf_pcb_sz_x, ttf_pcb_sz_y, wall+border_butt_sz_z], 2, true);
    }
    // ears holes
    translate([0, 0, ear_sz_y/4]) {
      for(x=[-1,1]) {
        translate([x*(ttf_pcb_sz_x+2*wall)/2, 0, 0]) {
            rotate([0, 90, 0]) {
              cylinder(d=hole_d, h=20, center=true);
            }
        }
      }
    }
    for(x=[-1,1]) {
      translate([x*20/2, 0, 0]) {
        cylinder(d=hole_d, h=20, center=true);
      }
    }
    // pins hole
    translate([(ttf_pcb_sz_x-pins_hole_sz_x)/2, 0, 0]) {
      cube([pins_hole_sz_x, pins_hole_sz_y, 20], center=true);
    }
  }
}

module pole_attachment() {
  difference() {
    translate([-17, -13, 0]) {
      connector(sz_x=30);
    }
    for(x=[-1,1]) {
      translate([x*20/2, 0, 0]) {
        cylinder(d=hole_d_through, h=20, center=true);
      }
    }
  }
}


translate([0, 0, border_sz_z+wall]) {
//  pole_attachment();
}

//ttf_face_part();
translate([0, 0, border_sz_z+wall]) {
  ttf_butt_part();
}
