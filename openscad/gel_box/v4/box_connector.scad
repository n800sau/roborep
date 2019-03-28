left_fwd = 0;
left_back = 0;
right_fwd = 0;
right_back = 1;

use <../../lib/vent.scad>

$fn = 50;

overlap = 15;
overlap_attach = 5;

wall = 2;
gap = 0.5;
hole_d_through = 4;
hole_d = 3.2;
hole_wall = 3;

meter_sz_x = 63.5;
meter_sz_y = 56;
meter_sz_z = 55.5;
meter_d = 49;
meter_hole_dist_x = 52;
meter_hole_offset_z = 11;

meter_box_sz_x = meter_sz_x + 2*wall + gap + overlap;
meter_box_sz_y = meter_sz_y + 2*wall + 2*gap;
meter_box_sz_z = meter_sz_z + 2*wall + 2*gap;

meter_box_hole_pos_y = [5, meter_box_sz_y-10];

mod_sz_x = 60;
mod_sz_y = 46;
mod_sz_z = 1.5;
mod_sz_z_under = 3;
mod_sz_z_above = 19;

mod_hole_dist_x = 53;
mod_hole_dist_y1 = 39.5;
mod_hole_dist_y2 = 30;
mod_hole_pos = [
  [0, 0],
  [-mod_hole_dist_x, 0],
  [-mod_hole_dist_x, -mod_hole_dist_y2],
  [0, -mod_hole_dist_y1]
];

mod_hole_wall = 2;

back_box_sz_x = meter_box_sz_x;
back_box_sz_y = meter_box_sz_y + wall;
back_box_mod_sz_z = mod_sz_z_under + mod_sz_z + mod_sz_z_above;
back_box_sz_z = meter_box_sz_z+wall;

socket_d = 4.5;
power12_d = 8.4;

module meter_stand_or_holes(d) {
  translate([0, meter_box_hole_pos_y[0], 0]) {
		cylinder(d=d, h=meter_box_sz_z, center=true);
	}
  translate([0, meter_box_hole_pos_y[1], 8/2]) {
		cylinder(d=d, h=meter_box_sz_z-8, center=true);
	}	
}

module meter_stand_with_holes() {
  difference() {
    meter_stand_or_holes(d=hole_d+2*hole_wall);
    meter_stand_or_holes(d=hole_d);
  }
}

module back_stand_or_holes(d) {
  translate([0, meter_box_hole_pos_y[0], 0]) {
		cylinder(d=d, h=back_box_mod_sz_z, center=true);
	}
  translate([0, meter_box_hole_pos_y[1], 0]) {
		cylinder(d=d, h=back_box_mod_sz_z, center=true);
	}	
}

module back_stand_with_holes() {
  difference() {
    back_stand_or_holes(d=hole_d+2*hole_wall);
    back_stand_or_holes(d=hole_d_through);
  }
}

module meter_center_stand_or_holes(d) {
  translate([meter_box_sz_x/2-overlap, meter_box_hole_pos_y[0], (meter_box_sz_z-15)/2]) {
		cylinder(d=d, h=15, center=true);
	}
  translate([0, meter_box_hole_pos_y[1], hole_wall/2+hole_d-meter_box_sz_z/2]) {
    translate([meter_box_sz_x/2-overlap, 0, 0]) {
      rotate([90, 0, 0]) {
        cylinder(d=d, h=20, center=true);
      }
    }
    translate([-meter_box_sz_x/2+2*hole_wall, 0, 0]) {
      rotate([90, 0, 0]) {
        cylinder(d=d, h=20, center=true);
      }
    }
	}
}

module meter_center_stand_with_holes() {
  difference() {
    meter_center_stand_or_holes(d=hole_d+2*hole_wall);
    meter_center_stand_or_holes(d=hole_d);
  }
}

module mod_stand_or_holes(d, h) {
  for(pos=mod_hole_pos) {
      translate([pos[0], pos[1], 0]) {
        cylinder(d=d, h=h, center=true);
      }
  }
}

module mod_stand_with_holes() {
  difference() {
    mod_stand_or_holes(d=hole_d+2*mod_hole_wall, h=mod_sz_z_above);
    mod_stand_or_holes(d=hole_d, h=mod_sz_z_above);
  }
}

module switch_holes() {
  cube([9.5, 5, 50], center=true);
  translate([9.5, 0, 0]) {
    cylinder(d=2.5, h = 50, center=true);
  }
  translate([-9.5, 0, 0]) {
    cylinder(d=2.5, h = 50, center=true);
  }
}

module back_center_holes() {
  translate([meter_box_sz_x/2-overlap, meter_box_hole_pos_y[0], (meter_box_sz_z-15)/2]) {
		cylinder(d=hole_d_through, h=100, center=true);
	}
  translate([0, meter_box_hole_pos_y[1], hole_wall/2+hole_d-meter_box_sz_z/2]) {
    translate([meter_box_sz_x/2-overlap, 0, 0]) {
      rotate([90, 0, 0]) {
        cylinder(d=hole_d_through, h=100, center=true);
      }
    }
    translate([-meter_box_sz_x/2+2*hole_wall, 0, 0]) {
      rotate([90, 0, 0]) {
        cylinder(d=hole_d_through, h=100, center=true);
      }
    }
	}
}

module meter_holes() {
  rotate([-90, 0, 0]) {
    cylinder(d=meter_d, h=meter_sz_y, center=true);
    for(xsgn=[-1,1]) {
      translate([xsgn*meter_hole_dist_x/2, meter_d/2-meter_hole_offset_z, 0]) {
        cylinder(d=hole_d_through, h=meter_sz_y, center=true);
      }
    }
  }
}

module attach_holes(d) {
  translate([0, meter_box_sz_y/4, 0]) {
    cylinder(d=d, h=meter_box_sz_z*2, center=true);
  }
  translate([0, -meter_box_sz_y/4, 0]) {
    cylinder(d=d, h=meter_box_sz_z*2, center=true);
  }
  translate([0, 0, meter_box_sz_z/4]) {
    rotate([90, 0, 0]) {
      cylinder(d=d, h=meter_box_sz_y*2, center=true);
    }
  }
  translate([0, 0, -meter_box_sz_z/4]) {
    rotate([90, 0, 0]) {
      cylinder(d=d, h=meter_box_sz_y*2, center=true);
    }
  }
}

module left_box_fwd() {
	difference() {
    cube([meter_box_sz_x, meter_box_sz_y, meter_box_sz_z], center= true);
		translate([wall, wall, wall]) {
			cube([meter_box_sz_x-wall, meter_box_sz_y-wall, meter_box_sz_z-wall], center= true);
		}
    translate([(meter_box_sz_x)/2-overlap+2*overlap_attach, wall/2, wall/2]) {
      attach_holes(d=hole_d_through);
    }
    translate([0, -meter_box_sz_y/2, 0]) {
      meter_holes();
    }
	}
  translate([-meter_box_sz_x/2+wall+(hole_wall+hole_d)/2, -meter_box_sz_y/2, 0]) {
    meter_stand_with_holes();
  }
  translate([0, -meter_box_sz_y/2, 0]) {
    meter_center_stand_with_holes();
  }
}

module right_box_fwd() {
	difference() {
    cube([meter_box_sz_x, meter_box_sz_y, meter_box_sz_z], center= true);
		translate([-wall, wall, wall]) {
			cube([meter_box_sz_x-wall, meter_box_sz_y-wall, meter_box_sz_z-wall], center= true);
		}
    translate([0, -meter_box_sz_y/2, 0]) {
      meter_holes();
    }
	}
  difference() {
    // overlap extension
    translate([-(meter_box_sz_x)/2-overlap/2+overlap_attach, wall/2, wall/2]) {
      difference() {
        cube([overlap, meter_box_sz_y-wall, meter_box_sz_z-wall], center= true);
        translate([0, wall, wall]) {
          cube([overlap, meter_box_sz_y-2*wall, meter_box_sz_z-2*wall], center= true);
        }
      }
    }
    translate([-(meter_box_sz_x)/2-overlap+2*overlap_attach, wall/2, wall/2]) {
      attach_holes(d=hole_d);
    }
  }
  translate([meter_box_sz_x/2-wall-(hole_wall+hole_d)/2, -meter_box_sz_y/2, 0]) {
    meter_stand_with_holes();
  }
  mirror([1, 0, 0]) {
    translate([0, -meter_box_sz_y/2, 0]) {
      meter_center_stand_with_holes();
    }
  }
}

module left_box_back() {
  difference() {
    cube([back_box_sz_x, back_box_sz_y, back_box_sz_z], center= true);
    translate([0, -wall, -wall]) {
      cube([back_box_sz_x, back_box_sz_y, back_box_sz_z], center= true);
    }
    translate([(meter_box_sz_x)/2-overlap+2*overlap_attach, wall/2, wall/2]) {
      attach_holes(d=hole_d_through);
    }
    translate([-back_box_sz_x/2+wall+(hole_wall+hole_d)/2, -back_box_sz_y/2, back_box_sz_z/2]) {
      back_stand_or_holes(d=hole_d_through);
    }
    translate([0, -meter_box_sz_y/2-wall/2, -wall/2]) {
      back_center_holes();
    }
    // high voltage GND socket
    translate([-back_box_sz_x/2+9, -12, (back_box_sz_z-mod_sz_z_above)/2-wall]) {
      cylinder(d=socket_d, h=50);
    }
    // 12v power hole
    translate([-19, 10, (back_box_sz_z-mod_sz_z_above)/2-wall]) {
      cylinder(d=power12_d, h=50);
    }
    translate([3, 10, back_box_sz_z/2]) {
      rotate([0, 0, 90]) {
        switch_holes();
      }
    }
  }
}

module right_box_back() {
  difference() {
    cube([back_box_sz_x, back_box_sz_y, back_box_sz_z], center= true);
    translate([0, -wall, -wall]) {
      cube([back_box_sz_x, back_box_sz_y, back_box_sz_z], center= true);
    }
    translate([back_box_sz_x/2-wall-(hole_wall+hole_d)/2, -back_box_sz_y/2, back_box_sz_z/2]) {
      back_stand_or_holes(d=hole_d_through);
    }
    mirror([1, 0, 0]) {
      translate([0, -meter_box_sz_y/2-wall/2, -wall/2]) {
        back_center_holes();
      }
    }
    // high voltage + socket
    translate([back_box_sz_x/2-9, -12, (back_box_sz_z-mod_sz_z_above)/2-wall]) {
      cylinder(d=socket_d, h=50);
    }
    // control hole (fixed)
    translate([-15, -13, back_box_sz_z/2]) {
      cylinder(d=6, h=50, center=true); 
    }
    translate([0, (back_box_sz_y-wall)/2, (back_box_sz_z-25)/2-wall]) {
      vent(x_size=60, z_size=20, y_size=wall, x_count=10, z_count=3, negate=true);
    }
    translate([-2, (back_box_sz_y-30)/2, (back_box_sz_z-wall)/2]) {
      rotate([90, 0, 0]) {
        vent(x_size=40, z_size=20, y_size=wall, x_count=10, z_count=3, negate=true);
      }
    }
  }
  difference() {
    union() {
      // overlap extension
      translate([-(back_box_sz_x)/2-overlap/2+overlap_attach, wall, wall]) {
        difference() {
          cube([overlap, back_box_sz_y-4*wall, back_box_sz_z-4*wall], center= true);
          translate([0, -2*wall, -wall]) {
            cube([overlap, back_box_sz_y-2*wall, back_box_sz_z-4*wall], center= true);
          }
        }
      }
    }
    translate([-(meter_box_sz_x)/2-overlap+2*overlap_attach, wall/2, wall/2]) {
      attach_holes(d=hole_d);
    }
  }
  // pcb
  translate([24, 21, (back_box_sz_z-mod_sz_z_above)/2-wall]) {
    mod_stand_with_holes();
  }
}

if(left_fwd) {
  translate([-meter_box_sz_x/2, 0, 0]) {
    left_box_fwd();
  }
}
if(left_back) {
  translate([-meter_box_sz_x/2, 1, (back_box_sz_z-meter_box_sz_z)/2]) {
    //color("red")
      left_box_back();
  }
}

if(right_fwd) {
  translate([meter_box_sz_x/2, 0, 0]) {
    //color("blue")
      right_box_fwd();
  }
}
if(right_back) {
  translate([meter_box_sz_x/2, 1, (back_box_sz_z-meter_box_sz_z)/2]) {
    //color("red")
      right_box_back();
  }
}
