use <../lib/lcd16x2.scad>
use <../lib/ear.scad>

$fn = 36;

wall = 2;
gap = 0.5;
hole_d = 3.2;
hole_d_through = 4;

pcb_sz_x = 70.5;
pcb_sz_y = 50.5;
pcb_sz_z = 1.5;
pcb_free_side_border = 2;
pcb_space_under = 3.5;

pcb_lcd_out_dist_y = 9;
pcb_lcd_disp_dist_z = 14;

ard_pcb_sz_x = 47.8;
ard_pcb_sz_y = 18;
ard_pcb_sz_z = 1.8;

ard_pcb_usb_side_hole_dist = 15;
ard_pcb_usb_side_hole_d = 2;

// center
ard_pcb_usb_sz_x = 2.2;
ard_pcb_usb_sz_y = 8;

batt_sz_x = 58.2;
batt_wire_gap_x = 2.5;
batt_border_sz_x = 2;

batt_sz_y = 16.8;
batt_sz_z = 14;
// hole in center
batt_hole_d = 4;

lcd_sz_x = 80;
lcd_sz_y = 36;
lcd_sz_z = 1.5;

lcd_wall_sz_x = 90;
lcd_wall_sz_y = 60;

pcb_lcd_left_off_x = 2.3;
pcb_lcd_shift_x = (lcd_sz_x - pcb_sz_x)/2 - pcb_lcd_left_off_x;
pcb_lcd_stand_sz_y = pcb_lcd_out_dist_y - lcd_sz_z - wall - gap;

lcd_pcb_sz_z = 9;
// display x - center 1602A
lcd_display_pins_off_y = 7;
lcd_display_top_off_y = 5;

lcd_hole_dist_x = 75;
lcd_hole_dist_y = 31;

// termistors
// 100k ntc 3950
// ds18b20
// mf52at 100k 3950
// ntc 10k 3435 mf5b

bolt_connector_count = 3;

pin_dist = 2.75;

bolt_connector_sz_x = 11;
bolt_connector_space_sz_x = pin_dist*5;
bolt_area_sz_x = bolt_connector_count * bolt_connector_space_sz_x;
bolt_connector_sz_y = 7.5;
bolt_connector_sz_z = 10 + 50;
bolt_connector_pin_sz_z = 4.5;

proto_pcb_sz_x = 70;
proto_pcb_sz_y = 50.5;
proto_pcb_sz_z = 1.2;

bolt_connector_pcb_x = proto_pcb_sz_x;
bolt_connector_pcb_y = proto_pcb_sz_y;
bolt_connector_pcb_z = 1.2;

au_connector_d = 6;

box_sz_x = max(lcd_sz_x, proto_pcb_sz_x, batt_sz_x) + 10;
box_sz_y = max(lcd_sz_y, proto_pcb_sz_y, batt_sz_y) + 10;
box_sz_z = 10 + batt_sz_z + 10;

module ear(base_dist, height, d=3, cone=false) {
	bolt_hole_cone_center(base_dist=base_dist, hole_d=d, hole_wall=3, height=height, hole_height=cone ? height/2 : height);
}

module ear_holes(d, h=50, base_dist=3, ear_mode=false, cone=false, height=7) {
	for(xsgn=[-1, 1]) {
		for(ysgn=[-1, 1]) {
			translate([xsgn*box_sz_x/2, ysgn*box_sz_y/2, 0]) {
				if(ear_mode) {
					angle = cone ?
						(xsgn > 0 ? 
							(
								(ysgn > 0) ? -135 : 135
							) :
								(ysgn > 0) ? -45 : 45
						) :
						(xsgn > 0 ? 0 : 180);
					rotate([cone ? 0 : 180, 0, angle]) {
						ear(base_dist=base_dist, height=height, d=d, cone=cone);
					}
				} else {
					cylinder(d=d, h=h, center=true);
				}
			}
		}
	}
}

module top_lid() {
	difference() {
		translate([0, 0, (box_sz_z+wall)/2]) {
			difference() {
				cube([box_sz_x, box_sz_y, wall], center=true);
				lcd_box_hole(h=50);
				lcd_holes(h=50, d=hole_d_through);
			}
			translate([-wall, (box_sz_y-bolt_connector_sz_y+wall)/2, -(box_sz_z-10)/2]) {
				cube([bolt_area_sz_x+2*wall, bolt_connector_sz_y+wall, box_sz_z-10], center=true);
			}
		}
		translate([0, (box_sz_y-proto_pcb_sz_y)/2, -box_sz_z/2+5]) {
			bolt_connectors();
		}
		ear_holes(d=hole_d);
	}
}

module a_bolt_connector() {
	cube([bolt_connector_sz_x, bolt_connector_sz_y, bolt_connector_sz_z], center=true);
}

module bolt_connectors() {
	translate([-(bolt_area_sz_x-bolt_connector_sz_x)/2, (proto_pcb_sz_y-bolt_connector_sz_y)/2, bolt_connector_sz_z/2]) {
		for(i=[0:bolt_connector_count-1]) {
			translate([i*bolt_connector_space_sz_x, wall, 0]) {
				a_bolt_connector();
			}
		}
	}
	// pcb
	cube([proto_pcb_sz_x, proto_pcb_sz_y, proto_pcb_sz_z], center=true);
}

module bottom_box() {
	translate([0, (box_sz_y-proto_pcb_sz_y)/2, 5]) {
		%bolt_connectors();
	}
	translate([0, 0, wall/2]) {
		cube([box_sz_x, box_sz_y, wall], center=true);
		translate([0, (box_sz_y-wall)/2, 5/2]) {
			cube([box_sz_x, wall, 5], center=true);
		}
		// back wall
		translate([0, -(box_sz_y-wall)/2, box_sz_z/2]) {
			cube([box_sz_x, wall, box_sz_z], center=true);
		}
		// side walls
		for(xsgn=[-1,1]) {
			translate([xsgn*box_sz_x/2, 0, box_sz_z/2]) {
				cube([wall, box_sz_y, box_sz_z], center=true);
			}
		}
	}
}

module pcb_holder() {
  for(xsgn=[-1,1]) {
    translate([xsgn*pcb_sz_x/2, pcb_lcd_out_dist_y/2, -pcb_space_under/2]) {
      // bottom part
      cube([2*pcb_free_side_border, pcb_sz_y+pcb_lcd_out_dist_y, pcb_space_under], center=true);
      translate([xsgn*pcb_free_side_border/2, 0, (pcb_space_under+pcb_sz_z)/2]) {
        // middle part
        cube([pcb_free_side_border, pcb_sz_y+pcb_lcd_out_dist_y, pcb_sz_z], center=true);
      }
      translate([0, 0, (pcb_space_under+wall)/2+pcb_sz_z]) {
        // top part
        cube([2*pcb_free_side_border, pcb_sz_y+pcb_lcd_out_dist_y, wall], center=true);
      }
    }
  }
  translate([0, (pcb_sz_y+wall)/2, 0]) {
    cube([pcb_sz_x, wall, pcb_space_under+pcb_sz_z+wall], center=true);
  }
  translate([0,, (pcb_sz_y-wall)/2+pcb_lcd_out_dist_y, 0]) {
    translate([0, 0, (lcd_hole_dist_y+pcb_lcd_disp_dist_z+pcb_sz_z-pcb_space_under)/2]) {
      cube([lcd_wall_sz_x, wall, lcd_wall_sz_y], center=true);
    }
    translate([pcb_lcd_shift_x, 0, (lcd_hole_dist_y+pcb_lcd_disp_dist_z+pcb_sz_z-pcb_space_under)/2]) {
      difference() {
        union() {
          cube([lcd_sz_x, wall, lcd_hole_dist_y+pcb_lcd_disp_dist_z+pcb_sz_z+pcb_space_under], center=true);
          translate([0, -(pcb_lcd_stand_sz_y+wall)/2, pcb_lcd_disp_dist_z/2]) {
            rotate([90, 0, 0]) {
                lcd_holes(h=pcb_lcd_stand_sz_y, d=hole_d_through+2*wall);
            }
          }
        }
        translate([0, 0, pcb_lcd_disp_dist_z/2]) {
          rotate([90, 0, 0]) {
              lcd_box_hole(h=50);
              lcd_holes(h=50, d=hole_d_through);
          }
        }
      }
    }
  }
}

//top_lid();
//bottom_box();
pcb_holder();
