$fn = 50;

wall = 3;

ext_sz_x = 110;
ext_sz_y = 90;
ext_sz_z = 50;

int_sz_y = ext_sz_y - 2 * wall;


acryl_sz_x = 100;
acryl_sz_y = 80;
acryl_sz_z = 2;

bottom_sz_z = acryl_sz_z + wall;

insert_x_pos = [-ext_sz_x/2+15, -ext_sz_x/2+25, 0, ext_sz_x/2-25, ext_sz_x/2-15];
insert_sz_x = 6;
insert_sz_y = (ext_sz_y - int_sz_y)/2;
insert_sz_z = 3;

module gel_container() {
	difference() {
		cube([ext_sz_x, ext_sz_y, ext_sz_z], center = true);
		union() {
			for(xpos=insert_x_pos) {
				for(ysgn=[-1, 1]) {
					// top inserts
					translate([xpos, ysgn * (ext_sz_y-insert_sz_y)/2, (ext_sz_z-insert_sz_z)/2]) {
						cube([insert_sz_x, insert_sz_y, insert_sz_z], center = true);
					}
				}
			}
			// internal emptiness
			translate([0, 0, bottom_sz_z + handle_sz_z/2]) {
				cube([ext_sz_x, int_sz_y, ext_sz_z + handle_sz_z], center = true);
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

gel_container();
