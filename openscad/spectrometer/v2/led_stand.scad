$fn = 30;

wall = 2;

vrod_d = 6;
vrod_sz_z = 100;

vrod_base_d = 30;
vrod_base_h = 10;

hrod_lock_d = vrod_d + 2 * wall;
hrod_lock_sz_z = 10;
hrod_sz_y = 60;

module vrod() {
	translate([0, 0, vrod_base_h]) {
		cylinder(d=vrod_d, h=vrod_sz_z);
	}
	cylinder(d2=vrod_d, d1=vrod_base_d, h=vrod_base_h);
}

module hrod() {
	difference() {
		union() {
			cylinder(d=hrod_lock_d, h=hrod_lock_sz_z);
			translate([-hrod_lock_d, -wall*1.5, 0]) {
				cube([hrod_lock_d, 3*wall, hrod_lock_sz_z]);
			}
			translate([hrod_lock_d/2-wall, -hrod_sz_y/2, 0]) {
				cube([wall, hrod_sz_y, hrod_lock_sz_z]);
			}
			translate([-hrod_lock_d, -wall*0.5, (hrod_lock_d*3)/2]) {
				rotate([90, 0, 0]) {
					cylinder(d=4, h=hrod_lock_d*3);
				}
			}
		}
		cylinder(d=vrod_d, h=hrod_lock_sz_z);
		translate([-hrod_lock_d, -wall*0.5, 0]) {
			cube([hrod_lock_d, wall, hrod_lock_sz_z]);
		}
	}
}

//vrod();
//translate([0, 0, vrod_sz_z/2]) {
	hrod();
//}

