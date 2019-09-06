$fn = 50;

wall = 2;
out_d = 70;
hole_dist = 75;
hole_d = 3.8;

difference() {
	union() {
		cylinder(d=out_d, h=wall, center=true);
		cube([20+2*wall, 18.5+2*wall, 12], center=true);
	}
	cube([20, 18.5, 50], center=true);
	for(hole_pos=[[1,-1], [1,1], [-1,1], [-1,-1]]) {
		translate([hole_pos[0]*hole_dist/2, hole_pos[1]*hole_dist/2, 0]) {
			cylinder(d=hole_d, h=wall, center=true);
		}
	}
}
