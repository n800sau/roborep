include <bb_box_params.scad>;
use <nuts_and_bolts.scad>;

//44 betwen holes

module lidlock(cyl_r, with_hole) {
	difference() {
		union() {
			translate([0, 0, -ll_thickness]) {
				cube([ll_width, ll_length, ll_thickness], center=true);
			}
			if(!with_hole) {
				translate([0, -ll_length/2 + 6, -ll_thickness/2+cyl_height/2]) {
					sphere(d=cyl_r*2, center=true);
				}
				translate([0, -ll_length/2 + 6, -1-ll_thickness/2+cyl_height/2]) {
					cylinder(r=cyl_r, h=cyl_height, center=true);
				}
			}
		}
		if(with_hole) {
			translate([0, -ll_length/2 + 6, -ll_thickness/2+cyl_height/2]) {
				cylinder(r=cyl_r, h=20, center=true);
			}
			translate([0, 0, -5]) {
				scale([1.2, 1.2, 2]) {
					mNut(center=true);
				}
			}
		}
	}
}

module RoundedBBLid() {

	full_width = bb_width + power_side_width +
    other_side_width - bb_curve_corner + wall_thickness+1.5;
  echo("Full Width:", full_width);
	full_height = bb_height-bb_curve_corner+wall_thickness*2;
	minkowski() {
		cube([full_width+ll_thickness*2, full_height, bblid_thickness],center=true);
		cylinder(r=bb_curve_corner/2, height=bblid_thickness);
	}

	translate([full_width/2+2,-bb_height/4, ll_length/2+bblid_thickness/2]) {
		rotate([90, 0, 0]) {
      rotate([0, -90, 0]) {
        lidlock(hole3m_radius-0.3, false);
      }
		}
	}

	translate([full_width/2+2, bb_height/4, ll_length/2+bblid_thickness/2]) {
		rotate([90, 0, 0]) {
      rotate([0, -90, 0]) {
        lidlock(hole3m_radius-0.3, false);
      }
		}
	}

	translate([-(full_width/2+2), -bb_height/4, ll_length/2+bblid_thickness/2]) {
		rotate([90, 0, 0]) {
      rotate([0, 90, 0]) {
        lidlock(hole3m_radius, true);
      }
    }
	}

	translate([-(full_width/2+2), bb_height/4, ll_length/2+bblid_thickness/2]) {
		rotate([90, 0, 0]) {
      rotate([0, 90, 0]) {
        lidlock(hole3m_radius, true);
      }
    }
	}

}

usb_l = 93;
usb_h = 26;
usb_w = 13;

a_l = 10;
a_w = 32.5;
a_h = 17;
a_th = 3;
a_hole_d = 3.1;
lid_hole_d = 4;
a_hole_w = 8;

module usb_attachment() {
  difference() {
    union(left_side=false, right_side=false) {
      // top
      cube([a_w, a_l, a_th]);
      translate([a_w-a_th+0.5, 0, a_th]) {
        // lock
        rotate([-90, 0, 0]) {
          cylinder(r=2.5, h=a_l);
        }
      }
      hull() {
        translate([-3, 0, 0]) {
          cube([3, a_l, a_th]);
        }
        // wall
        cube([a_th, a_l, a_h]);
        translate([-a_hole_w, 0, a_h-a_th]) {
          // attach
          cube([a_hole_w, a_l, a_th]);
        }
      }
    }
    translate([-a_hole_w/5, a_l/2, 10]) {
      cylinder(d=a_hole_d, h=11);
      translate([0, 0, 6]) {
        cylinder(d=a_hole_d+0.2, h=17);
      }
    }
  }
}

module BBLid(show_lid=true, attach_n=[true, true, true]) {
  pos = [5, bb_height/2-a_l/2, bb_height-a_l-5];
	if(show_lid) {
		difference() {
			RoundedBBLid();
			for(i=[0:2]) {
				translate([-10, -bb_height/2 + pos[i], -10]) {
					translate([-a_hole_w/5, a_l/2, 0]) {
						cylinder(d=lid_hole_d, h=20);
					}
				}
			}
		}
	}
  if(attach_n[0])
    translate([-10, -bb_height/2 + pos[0], -20]) {
      usb_attachment(left_side=true);
    }
  if(attach_n[1])
    translate([-10, -bb_height/2 + pos[1], -20]) {
      usb_attachment();
    }
  if(attach_n[2])
    translate([-10, -bb_height/2 + pos[2], -20]) {
      usb_attachment(right_side=true);
    }
}

BBLid(show_lid=true);
