$fn = 50;

wall = 2;
gap = 0.5;

mod_sz_x = 40;
mod_sz_y = 30;
mod_sz_z = 2;
mod_sz_z_under = 3;
mod_sz_z_above = 15;

hole_dist_x = 38;
hole_dist_y = 28;
hole_d_through = 4;
hole_d = 3.2;
hole_wall = 3;

screw_hole_x = 12;
screw_hole_y = 7;
screw_hole_d = 10;

bottom_box_ext_x = mod_sz_x+2*wall+2*gap;
bottom_box_ext_y = mod_sz_y+2*wall+2*gap;
bottom_box_ext_z = wall+gap+mod_sz_z_under+mod_sz_z;
bottom_box_int_x = mod_sz_x+2*gap;
bottom_box_int_y = mod_sz_y+2*gap;
bottom_box_int_z = bottom_box_ext_z - wall;
bottom_box_stand_z = wall+gap+mod_sz_z_under;

top_box_ext_x = bottom_box_ext_x;
top_box_ext_y = bottom_box_ext_y;
top_box_ext_z = wall+gap+mod_sz_z_above;
top_box_int_x = bottom_box_int_x;
top_box_int_y = bottom_box_int_y;
top_box_int_z = top_box_ext_z - wall;
top_box_stand_z = top_box_int_z;

module stand_or_holes(d, h) {
	for(xsgn=[-1, 1]) {
		for(ysgn=[-1, 1]) {
			translate([xsgn*hole_dist_x/2, ysgn*hole_dist_y/2, 0]) {
				cylinder(d=d, h=h, center=true);
		}
	}
}

module top_box() {
	difference() {
		cube([top_box_ext_x, top_box_ext_y, top_box_ext_z], center=true);
		translate([0, 0, (top_box_int_z-top_box_ext_z)/2-wall]) {
			cube([top_box_int_x, top_box_int_y, top_box_int_z], center=true);
		}
		stand_or_holes(d=hole_d_through, h=top_box_ext_z);
	}
}

module bottom_box() {
	difference() {
		union() {
			cube([bottom_box_ext_x, bottom_box_ext_y, bottom_box_ext_z], center=true);
			translate([0, 0, (bottom_box_stand_z-bottom_box_ext_z)/2]) {
				stand_or_holes(d=hole_d_through+2*hole_wall, h=bottom_box_stand_z);
			}
		}
		translate([0, 0, (bottom_box_int_z-bottom_box_ext_z)/2+wall]) {
			cube([bottom_box_int_x, bottom_box_int_y, bottom_box_int_z], center=true);
		}
		stand_or_holes(d=hole_d_through, h=bottom_box_ext_z);
	}
}

module thread_stands() {
	difference() {
		stand_or_holes(d=hole_d_through+2*hole_wall, h=top_box_stand_z-2*gap);
		stand_or_holes(d=hole_d, h=top_box_stand_z);
	}
}


translate([0, 0, bottom_box_ext_z+top_box_ext_z/2+gap]) {
	top_box();
}
translate([0, 0, bottom_box_ext_z+top_box_stand_z/2+gap]) {
	thread_stands();
}
translate([0, 0, bottom_box_ext_z/2]) {
	bottom_box();
}

