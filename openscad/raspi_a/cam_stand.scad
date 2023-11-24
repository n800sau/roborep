pad = 0.5;
wall = 3;
base_sz_x = 60;
base_sz_y = 60;
base_sz_z = wall;

dist_y = 12;

pole_d = 12;
pole_sz_z = 270;

pole_socket_int_d = pole_d + pad;
pole_socket_ext_d = pole_d + 2*wall;
pole_socket_sz_z = 50;

clip_sz_z = 10;
clip_wall = 3;
clip_ear_sz_x = wall;
clip_ear_sz_y = 10+wall;
clip_gap_sz_x = 3;

clip_pole_d = 6;
clip_pole_sz = 20;

rpi_plate_sz_x = 80;
rpi_plate_sz_y = 89;
rpi_plate_sz_z = wall;
hole_dist_x = 68;
hole_dist_y = 79;

rpi_plate_cut_sz_x = 40;
rpi_plate_cut_sz_y = 40;
rpi_plate_cut_sz_z = wall;

rpi_pole_d = 6;
rpi_pole_sz_y = 200;

module pole() {
	translate([0, 0, base_sz_z/2]) {
//		cube([base_sz_x, base_sz_y, base_sz_z], center=true);
	}
	translate([0, 0, (pole_sz_z+base_sz_z)/2]) {
		cylinder(d=pole_d, h=pole_sz_z, center=true);
	}
}

module pole_socket() {
	difference() {
  hull() {
    translate([0, 0, base_sz_z/2]) {
      cube([base_sz_x, base_sz_y, base_sz_z], center=true);
    }
    translate([0, 0, (pole_socket_sz_z+base_sz_z)/2]) {
			cylinder(d=pole_socket_ext_d, h=pole_socket_sz_z, center=true);
    }
	}
    translate([0, 0, (pole_socket_sz_z+base_sz_z)/2+pole_socket_sz_z-30]) {
			cylinder(d=pole_socket_int_d, h=pole_socket_sz_z, center=true);
    }
  }
}


module clip() {
	difference() {
		union() {
			cylinder(d=pole_d+pad*2+clip_wall*2, h=clip_sz_z, center=true);
			for(sgn=[-1,1]) {
				translate([sgn*clip_gap_sz_x/2, (pole_d+pad*2)/2, 0]) {
					cube([clip_ear_sz_x, clip_ear_sz_y, clip_sz_z], center=true);
				}
			}
			translate([0, clip_pole_sz/2, 0]) {
				rotate([90, 0, 0]) {
					cylinder(d=pole_d, h=clip_pole_sz, center=true);
				}
			}
		}
		cylinder(d=pole_d+pad*2, h=clip_sz_z, center=true);
	}
}

module rpi_plate() {
	difference() {
		union() {
			cube([rpi_plate_sz_x, rpi_plate_sz_y, rpi_plate_sz_z], center=true);
			rotate([90, 0, 0]) {
				cylinder(d=rpi_pole_d, h=rpi_pole_sz_y, center=true);
			}
		}
		cube([rpi_plate_cut_sz_x, rpi_plate_cut_sz_y, rpi_plate_cut_sz_z], center=true);
		translate([0, 0, (rpi_plate_sz_z+rpi_pole_d)/2]) {
			cube([rpi_plate_sz_x, rpi_plate_sz_y, rpi_pole_d], center=true);
		}
		for(pos=[
					[10, 10],
					[50, 10],
					[50, 50],
					[10, 50]
				]) {
			translate([pos[0], pos[1], 10]) {
				cylinder(d=3.2, h=20, center=true);
			}
		}
	}
}

for(sgn=[1]) {
	translate([0, sgn*(dist_y+base_sz_y)/2, 0]) {
		pole_socket();
//		pole();
		translate([0, 0, 20]) {
			rotate([0, 0, (sgn == 1)*180]) {
//				clip();
			}
		}
	}
}

translate([0, 0, 30]) {
//	rpi_plate();
}


