$fn = 50;

wall = 2;
ext_d = 31.4;
h = 12;
wire_hole_sz_y = 5;

//union() {
difference() {
	cylinder(d=ext_d, h=h);
	translate([0, 0, wall]) {
		cylinder(d=ext_d-2*wall, h=h-wall);
	}
	translate([ext_d/4, -wire_hole_sz_y/2, h-wall]) {
		cube([ext_d/2, wire_hole_sz_y, wall]);
	}
}
translate([-(ext_d+10)/2, -5, 0]) {
	difference() {
		union() {
			cube([ext_d+10, 10, wall]);
			translate([0, 5, 0]) {
				cylinder(d=10, h=wall);
			}
			translate([ext_d+10, 5, 0]) {
				cylinder(d=10, h=wall);
			}
		}
		translate([0, 5, 0]) {
			cylinder(d=3.5, h=wall);
		}
		translate([ext_d+10, 5, 0]) {
			cylinder(d=3.5, h=wall);
		}
	}
}
