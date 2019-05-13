$fn = 50;

wall = 2;
gap = 0.5;

box_sz_x = 150;
box_sz_y = 50;
box_sz_z = 50;

window_sz_y = 20;
window_sz_z = 20;

cam_window_sz_y = 30;
cam_window_sz_z = 30;

cam_wall_offset = 20;
cam_wall_angle = 30;
cam_wall_sz_z = box_sz_z / cos(cam_wall_angle);
cam_wire_sz_z = 5;

middle_wall_offset = 50;
middle_insert_hole_sz_x = wall + gap;

slot_sz_y = 30;
slot_sz_z = 1;

module sp_base() {
	// bottom
	translate([0, 0, -(box_sz_z+wall)/2]) {
		cube([box_sz_x, box_sz_y, wall], center=true);
	}
	// side walls
	for(ysgn = [-1, 1]) {
		translate([0, ysgn*(box_sz_y+wall)/2, 0]) {
			cube([box_sz_x, wall, box_sz_z], center=true);
		}
	}
	// window wall
	translate([(box_sz_x+wall)/2, 0, 0]) {
		difference() {
			cube([wall, box_sz_y, box_sz_z], center=true);
			cube([wall, window_sz_y, window_sz_z], center=true);
		}
	}
	// middle insert holder
	translate([(box_sz_x+wall)/2-middle_wall_offset, 0, 0]) {
		for(xsgn=[-1, 1]) {
			translate([xsgn*(middle_insert_hole_sz_x+wall)/2, 0, 0]) {
				difference() {
					cube([wall, box_sz_y, box_sz_z], center=true);
					cube([wall, window_sz_y, window_sz_z], center=true);
				}
			}
		}
	}
	// camera wall
	translate([-(box_sz_x+wall)/2+cam_wall_offset, 0, 0]) {
		rotate([0, -cam_wall_angle, 0]) {
			difference() {
				cube([wall, box_sz_y, box_sz_z], center=true);
				cube([wall, cam_window_sz_y, cam_window_sz_z], center=true);
			}
		}
	}
}

module top_lid() {
	// top
	translate([-middle_wall_offset/2, 0, (box_sz_z+wall)/2]) {
		cube([box_sz_x-middle_wall_offset, box_sz_y, wall], center=true);
	}
	translate([-(box_sz_x+wall)/2, 0, box_sz_z-cam_wire_sz_z]) {
		cube([wall, box_sz_y, box_sz_z-cam_wire_sz_z], center=true);
	}
}

module slot_insert() {
	difference() {
		cube([wall, box_sz_y, box_sz_z], center=true);
		cube([wall, slot_sz_y, slot_sz_z], center=true);
	}
}


top_lid();
sp_base();
translate([middle_wall_offset, 0, 0]) {
//	slot_insert();
}

