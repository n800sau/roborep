$fn = 40;

sz_x = 12;
sz_y = 20;
sz_z = 118+23;

hole_m4_d = 4;
hole_m3_d_through = 3.6;
wall = 3;

difference() {
	union() {
		cube([sz_x, wall, sz_z-sz_x/2]);
    translate([sz_x/2, wall, sz_z-sz_x/2]) {
      rotate([90, 0, 0]) {
        cylinder(d=sz_x, h=wall);
      }
    }
    cube([sz_x, sz_y+wall-sz_x/2, wall]);
    translate([sz_x/2, sz_y+wall-sz_x/2, 0]) {
      cylinder(d=sz_x, h=wall);
    }
	}
	for(z=[0, 116]) {
		translate([sz_x/2, 0, 15+z]) {
			rotate([-90, 0, 0]) {
				cylinder(d=hole_m4_d, h=10);
			}
		}
	}
	translate([sz_x/2, sz_y+wall-hole_m3_d_through-wall, 0]) {
		cylinder(d=hole_m3_d_through, h=10);
	}
}

