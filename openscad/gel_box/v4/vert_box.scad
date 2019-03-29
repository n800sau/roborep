vertical_fwd = 0;
vertical_back = 1;

use <../../lib/vent.scad>
use <../../lib/ear.scad>

$fn = 50;

overlap = 15;
overlap_attach = 5;

wall = 3;
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
meter_box_sz_z = 2*meter_sz_z + 2*gap + 2*wall + 2*gap;

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
	[0, 0, true],
	[-mod_hole_dist_x, 0, true],
	[-mod_hole_dist_x, -mod_hole_dist_y2, false],
	[0, -mod_hole_dist_y1, true]
];

mod_hole_wall = 2;

back_box_sz_x = meter_box_sz_x;
back_box_sz_y = meter_box_sz_y + wall;
back_box_mod_sz_z = mod_sz_z_under + mod_sz_z + mod_sz_z_above;
back_box_sz_z = meter_box_sz_z+wall;

socket_d = 4.5;
power12_d = 8.4;

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

module switch_holes() {
	cube([9.5, 5, 50], center=true);
	translate([9.5, 0, 0]) {
		cylinder(d=2.5, h = 50, center=true);
	}
	translate([-9.5, 0, 0]) {
		cylinder(d=2.5, h = 50, center=true);
	}
}

module mod_stand_or_holes(stand=true) {
	h = mod_sz_z_above;
	for(pos=mod_hole_pos) {
		translate([pos[0], pos[1], 0]) {
			translate([0, 0, (h-mod_sz_z_above)/2]) {
				if(stand) {
					d = pos[2] ? hole_d+2*mod_hole_wall : 2*mod_hole_wall;
				} else {
					d = hole_d;
				}
				cylinder(d=d, h=h, center=true);
			}
		}
	}
}

module mod_stand_with_holes() {
	difference() {
		mod_stand_or_holes(true);
		mod_stand_or_holes(false);
	}
}

module vertical_box_fwd() {
	difference() {
		union() {
			// wall left
			translate([(-wall-meter_box_sz_x)/2, 0, -wall/2]) {
				cube([wall, meter_box_sz_y+wall, meter_box_sz_z+wall], center= true);
			}
			// wall right
			translate([(meter_box_sz_x+wall)/2, -wall/2, -wall/2]) {
				cube([wall, meter_box_sz_y+wall, meter_box_sz_z+wall], center= true);
			}
			// bottom
			translate([0, -wall/2, (-wall-meter_box_sz_z)/2]) {
				cube([meter_box_sz_x+2*wall, meter_box_sz_y+2*wall, wall], center= true);
			}
			// face
			translate([0, (-wall-meter_box_sz_y)/2, -wall/2]) {
				cube([meter_box_sz_x+2*wall, wall, meter_box_sz_z+wall], center= true);
			}
		}
		translate([0, -meter_box_sz_y/2, (meter_sz_z-meter_box_sz_z)/2-gap]) {
			meter_holes();
		}
		translate([0, -meter_box_sz_y/2, meter_sz_z/2+gap]) {
			meter_holes();
		}
		// high voltage GND socket
		translate([0, -meter_box_sz_y/4, meter_box_sz_z/4]) {
			translate([-meter_box_sz_x/2, 0, 0]) {
				rotate([0, 90, 0]) {
					cylinder(d=socket_d, h=50, center=true);
				}
			}
			// high voltage + socket
			translate([meter_box_sz_x/2, 0, 0]) {
				rotate([0, 90, 0]) {
					cylinder(d=socket_d, h=50, center=true);
				}
			}
		}
		// 12v power hole
		translate([-meter_box_sz_x/2, meter_box_sz_y/4, 0]) {
			rotate([0, 90, 0]) {
				cylinder(d=power12_d, h=50, center=true);
			}
		}
		// switch
		translate([-meter_box_sz_x/2, meter_box_sz_y/4, (power12_d+60)/2]) {
			rotate([0, 90, 0]) {
				switch_holes();
			}
		}
	}
	for(xsgn=[-1, 1]) {
		for(ysgn=[-1, 1]) {
			translate([xsgn*(meter_box_sz_x-wall)/2, ysgn*(meter_box_sz_y/2-wall), meter_box_sz_z/2]) {
				rotate([0, 0, xsgn < 0 ? 180 : 0]) {
					bolt_hole_cone();
				}
			}
		}
	}
}

module vertical_box_back() {
	// back
	translate([0, (meter_box_sz_y+wall)/2, -wall/2]) {
		cube([meter_box_sz_x+2*wall, wall, meter_box_sz_z+2*wall], center= true);
	}
	difference() {
		// top
		translate([0, -wall/2, meter_box_sz_z/2]) {
			cube([meter_box_sz_x+2*wall, meter_box_sz_y+2*wall, wall], center= true);
		}
		for(xsgn=[-1, 1]) {
			for(ysgn=[-1, 1]) {
				translate([xsgn*(meter_box_sz_x-wall)/2, ysgn*(meter_box_sz_y/2-wall), meter_box_sz_z/2]) {
					cylinder(d=hole_d_through, h=100, center=true);
				}
			}
		}
	}
	// pcb
	translate([24, 21, (meter_box_sz_z-mod_sz_z_above)/2-wall]) {
		mod_stand_with_holes();
	}
}

if(vertical_fwd) {
	vertical_box_fwd();
}

if(vertical_back) {
	vertical_box_back();
}
