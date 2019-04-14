withwin = 0;
show_container = 0;
show_all = 0;
show_wall_insert = 0;
show_comb_insert = 1;

$fn = 50;

wall = 2;

ext_sz_x = 110;
//ext_sz_y = 86.2;
ext_sz_y = 85.8;
ext_sz_z = 40;

acryl_sz_x = 103;
acryl_sz_y = 82;
acryl_sz_z = 3;

bottom_sz_z = acryl_sz_z + wall;

int_sz_y = ext_sz_y - 2 * wall;
int_sz_z = ext_sz_z - bottom_sz_z;

insert_x_pos = [
  -ext_sz_x/2+8,
  -ext_sz_x/2+22, 
  0,
  ext_sz_x/2-22,
  ext_sz_x/2-8
];
insert_sz_x = 6;
insert_sz_y = (ext_sz_y - int_sz_y)/2;
insert_sz_z = int_sz_z - 6;
insert_handle_sz_z = 10;

comb_sz_x = 1.5;
comb_sz_z = int_sz_z - 5;
comb_handle_sz_z = 15;

gap = 0.2;

rubber_sz_z = 15;
rubber_sz_x = 1.5;

tooth_sz_z = 20;
tooth_sz_y = 5;
tooth_gap_y = 3;

insert_place_sz_z = max(insert_handle_sz_z, comb_handle_sz_z);

module gel_container() {
	difference() {
		cube([ext_sz_x, ext_sz_y, ext_sz_z], center = true);
		union() {
			for(xpos=insert_x_pos) {
				for(ysgn=[-1, 1]) {
					// top inserts
					translate([xpos, ysgn * (ext_sz_y-insert_sz_y)/2, (ext_sz_z-insert_place_sz_z)/2]) {
						cube([insert_sz_x+2*gap, insert_sz_y, insert_place_sz_z], center = true);
					}
				}
			}
			// internal emptiness
			translate([0, 0, bottom_sz_z+insert_place_sz_z/2]) {
				cube([ext_sz_x, int_sz_y, ext_sz_z+insert_place_sz_z], center = true);
			}
      if(withwin) {
        // acrylic place
        translate([0, 0, bottom_sz_z-acryl_sz_z]) {
  				cube([acryl_sz_x, acryl_sz_y, ext_sz_z], center = true);
        }
        // transparent hole
  			cube([acryl_sz_x-2*wall, acryl_sz_y-2*wall, ext_sz_z], center = true);
      }
    }
	}
}

module wall_insert() {
	// handle
	translate([0, 0, (int_sz_z-insert_handle_sz_z)/2]) {
		cube([insert_sz_x, ext_sz_y + 20, insert_handle_sz_z], center=true);
	}
	// thiner part whole height
  translate([(insert_sz_x-2*rubber_sz_x)/2, 0, 0]) {
    cube([insert_sz_x-2*rubber_sz_x, int_sz_y-2*gap, insert_sz_z], center=true);
  }
	// thicker part above rubber
	translate([0, 0, rubber_sz_z/2]) {
		cube([insert_sz_x, int_sz_y-2*gap, insert_sz_z-rubber_sz_z], center=true);
	}
}

module comb_insert() {
	difference() {
		union() {
			// handle
			translate([0, 0, (int_sz_z-comb_handle_sz_z)/2]) {
				cube([insert_sz_x, ext_sz_y + 20, comb_handle_sz_z], center=true);
			}
			// thiner part whole height
      translate([(insert_sz_x-comb_sz_x)/2, 0, 0]) {
        cube([comb_sz_x, int_sz_y-2*gap, comb_sz_z], center=true);
      }
		}
		for(ypos=[0:4]) {
			for(ysgn=[-1,1]) {
				translate([0, ysgn*(ypos*(tooth_sz_y+tooth_gap_y)), (-int_sz_z+tooth_sz_z)/2]) {
					cube([insert_sz_x, tooth_gap_y, tooth_sz_z], center=true);
				}
			}
		}
	}
}

if(show_container || show_all) {
  gel_container();
}
if(show_all) {
  for(xpos=[insert_x_pos[0], insert_x_pos[len(insert_x_pos)-1]]) {
    translate([xpos, 0, bottom_sz_z/2]) {
      wall_insert();
    }
  }
  for(xi=[1:len(insert_x_pos)-2]) {
    translate([insert_x_pos[xi], 0, bottom_sz_z/2]) {
      comb_insert();
    }
  }
}
if(show_comb_insert) {
  rotate([0, 90, 0]) {
    comb_insert();
  }
}

if(show_wall_insert) {
  translate([37, 0, 0]) {
    rotate([0, 90, 0]) {
      wall_insert();
    }
  }
}
