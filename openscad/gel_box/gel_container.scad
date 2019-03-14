$fn = 50;

wall = 3;

ext_sz_x = 110;
ext_sz_y = 90;
ext_sz_z = 50;

acryl_sz_x = 100;
acryl_sz_y = 80;
acryl_sz_z = 2;

bottom_sz_z = acryl_sz_z + wall;

int_sz_y = ext_sz_y - 2 * wall;
int_sz_z = ext_sz_z - bottom_sz_z;

insert_x_pos = [-ext_sz_x/2+15, -ext_sz_x/2+25, 0, ext_sz_x/2-25, ext_sz_x/2-15];
insert_sz_x = 6;
insert_sz_y = (ext_sz_y - int_sz_y)/2;
insert_sz_z = 3;

gap = 0.5;

rubber_sz_z = 6;
rubber_sz_x = 1.5;

tooth_sz_z = 20;
tooth_sz_y = 5;
tooth_gap_y = 3;

module gel_container() {
	difference() {
		cube([ext_sz_x, ext_sz_y, ext_sz_z], center = true);
		union() {
			for(xpos=insert_x_pos) {
				for(ysgn=[-1, 1]) {
					// top inserts
					translate([xpos, ysgn * (ext_sz_y-insert_sz_y)/2, (ext_sz_z-insert_sz_z)/2]) {
						cube([insert_sz_x+2*gap, insert_sz_y, insert_sz_z], center = true);
					}
				}
			}
			// internal emptiness
			translate([0, 0, bottom_sz_z + insert_sz_z/2]) {
				cube([ext_sz_x, int_sz_y, ext_sz_z + insert_sz_z], center = true);
			}
			// acrylic place
			translate([0, 0, bottom_sz_z-acryl_sz_z]) {
				cube([acryl_sz_x, acryl_sz_y, ext_sz_z], center = true);
			}
			// transparent hole
			cube([acryl_sz_x-2*wall, acryl_sz_y-2*wall, ext_sz_z], center = true);
		}
	}
}

module wall_insert() {
	// handle
	translate([0, 0, (ext_sz_z+insert_sz_z)/2]) {
		cube([insert_sz_x, ext_sz_y + 20, insert_sz_z], center=true);
	}
	// thiner part whole height
	cube([insert_sz_x-2*rubber_sz_x, int_sz_y-2*gap, int_sz_z], center=true);
	// thicker part above rubber
	translate([0, 0, (-int_sz_z + rubber_sz_z)/2]) {
		cube([insert_sz_x, int_sz_y-2*gap, rubber_sz_z], center=true);
	}
}

module comb() {
	difference() {
		union() {
			// handle
			translate([0, 0, (ext_sz_z+insert_sz_z)/2]) {
				cube([insert_sz_x, ext_sz_y + 20, insert_sz_z], center=true);
			}
			// thiner part whole height
			cube([insert_sz_x-2*rubber_sz_x, int_sz_y-2*gap, int_sz_z], center=true);
		}
		for(ypos=[0:3]) {
			for(ysgn=[-1,1]) {
				translate([0, ysgn*(ypos*(tooth_sz_y+tooth_gap_y)), (int_sz_z-tooth_sz_z)/2]) {
					cube([insert_sz_x, tooth_sz_y, tooth_sz_z], center=true);
				}
			}
		}
	}
}

gel_container();
for(xpos=[insert_x_pos[0], insert_x_pos[-1]]) {
	translate([xpos, 0, bottom_sz_z]) {
		wall_insert();
	}
}
for(xi=[1:len(insert_x_pos)-1]) {
	translate([insert_x_pos[xi], 0, bottom_sz_z]) {
		comb();
	}
}
