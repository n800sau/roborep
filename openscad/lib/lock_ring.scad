use <MCAD/nuts_and_bolts.scad>

$fn = 40;

module lock_ring(
	plus=true,
	wall=2,
	hrod_lock_d=14,
	hrod_lock_sz_z=10,
	vrod_d = 10,
  bolt_d=3
		) {
	if(plus) {
		union() {
			cylinder(d=hrod_lock_d, h=hrod_lock_sz_z);
			translate([-hrod_lock_d-wall, -wall*1.5, 0]) {
				cube([hrod_lock_d+wall, 3*wall, hrod_lock_sz_z]);
			}
		}
	} else {
		cylinder(d=vrod_d, h=hrod_lock_sz_z);
		translate([-hrod_lock_d-wall, -wall*0.5, 0]) {
			cube([hrod_lock_d+wall, wall, hrod_lock_sz_z]);
		}
		// bolt hole
		translate([-hrod_lock_d+bolt_d/2, hrod_lock_d*3/2, hrod_lock_d/2]) {
			rotate([90, 0, 0]) {
				cylinder(d=bolt_d, h=hrod_lock_d*3);
				translate([0, 0, hrod_lock_d*3/2-wall*2-1]) {
					nutHole(3);
				}
			}
		}
	}
}

difference() {
  lock_ring(plus=true);
  lock_ring(plus=false);
}
