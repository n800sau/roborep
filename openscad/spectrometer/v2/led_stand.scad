$fn = 30;

use <MCAD/nuts_and_bolts.scad>
use <pcb_box.scad>

wall = 2;

vrod_d = 6;
vrod_sz_z = 100;

vrod_base_d = 30;
vrod_base_h = 10;

hrod_lock_d = vrod_d + 2 * wall;
hrod_lock_sz_z = 10;
hrod_sz_y = 60;

bolt_d = 4;

led_leg_dist = 2.5;
led_d = 4.5;
led_leg_d = 1.5;

module vrod() {
	translate([0, 0, vrod_base_h]) {
		cylinder(d=vrod_d, h=vrod_sz_z);
	}
	cylinder(d2=vrod_d, d1=vrod_base_d, h=vrod_base_h);
}

module lock_ring(plus=true) {
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

module hrod() {
	difference() {
		union() {
			lock_ring();
			// led holder
			translate([hrod_lock_d/2-wall, -hrod_sz_y/2, 0]) {
				difference() {
					cube([wall, hrod_sz_y, hrod_lock_sz_z]);
					for(i=[0:9]) {
						translate([-5, 4+i*led_d+(i>=5 ? 12: 0), hrod_lock_sz_z/2+led_leg_dist/2]) {
							rotate([0, 90, 0]) {
								for(j=[0,1]) {
									translate([j*led_leg_dist, 0, 0]) {
										cylinder(d=led_leg_d, h=10);
									}
								}
							}
						}
					}
				}
			}
		}
		lock_ring(false);
	}
}

module vhrod() {
	difference() {
		union() {
			lock_ring();
			// led holder
			translate([hrod_lock_d/2-wall, -hrod_sz_y/2, 0]) {
				translate([0, hrod_sz_y/2, 0]) {
					cube([wall, 4, hrod_lock_sz_z]);
					translate([0, 4, -hrod_sz_y/2+5]) {
						difference() {
							cube([wall, hrod_lock_sz_z, hrod_sz_y]);
							for(i=[0:9]) {
								translate([-5, 0, 6+i*(led_d+0.5)]) {
									rotate([0, 90, 0]) {
										for(j=[0,1]) {
											translate([0, 4+j*led_leg_dist, 0]) {
												cylinder(d=led_leg_d, h=10);
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
		lock_ring(false);
	}
}

module control_support_box() {
	difference() {
		union() {
			lock_ring();
			translate([hrod_lock_d/2-wall, -30, 0]) {
				pcb_box_7x6(
					leg_sz_z=vrod_base_h+wall,
					leg_pos=[
						[-6, -6],
						[-6, 6]
					]
				);
			}
		}
		lock_ring(false);
	}
}

//vrod();
translate([0, 0, vrod_sz_z/2]) {
//	vhrod();
}
control_support_box();
