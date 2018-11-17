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

a_l = 10;
a_w = 20;
a_h = 15;
a_th = 3;
a_hole_d = 3.2;
a_hole_w = 10;

module usb_attachment() {
  union() {
    // top
    cube([a_w, a_l, a_th]);
    translate([0, 0, 0]) {
      hull() {
        // wall
        cube([a_th, a_l, a_h]);
        translate([-a_hole_w, 0, a_h-a_th]) {
          // attach
          cube([a_hole_w, a_l, a_th]);
        }
      }
    }
  }
}

module BBLid(show_lid=true, show_attach=true, single_attach=false) {
	step = bb_height/3;
	if(show_lid) {
		difference() {
			RoundedBBLid();
			for(i=[0:2]) {
				translate([-10, -bb_height/2 + step*(i+0.5), -10]) {
					translate([-a_w/5, a_l/2, 0]) {
						cylinder(d=a_hole_d, h=20);
					}
				}
			}
		}
	}
	if(show_attach) {
		attach_count = single_attach ? 0 : 2;
		for(i=[0:attach_count]) {
			translate([-10, -bb_height/2 + step*(i+0.5), -20]) {
				difference() {
					usb_attachment();
					translate([-a_w/5, a_l/2, 0]) {
						cylinder(d=a_hole_d, h=20);
					}
				}
			}
		}
	}
}
