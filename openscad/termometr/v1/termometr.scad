use <../../lib/lcd16x2.scad>
use <../../lib/ear.scad>
use <../../lib/switch.scad>

$fn = 36;

wall = 2;
gap = 0.25;
hole_d = 3.2;
hole_d_through = 4;

pcb_sz_x = 70.5;
pcb_sz_y = 50.5;
pcb_sz_z = 1.5;
pcb_free_side_border = 3;
pcb_space_under = 3.5;

box_sz_x = 100;
box_sz_y = pcb_sz_y + 2 * wall + 5;
box_sz_z = 53;

pcb_lcd_out_dist_y = 9;
pcb_lcd_dist_z = 8;

lcd_sz_x = 80;
lcd_sz_y = 36;
lcd_sz_z = 1.5;

pcb_lcd_left_off_x = 2.3;
pcb_lcd_shift_x = (lcd_sz_x - pcb_sz_x)/2 - pcb_lcd_left_off_x;
pcb_lcd_stand_sz_y = lcd_sz_z + wall/2 - gap;

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
bolt_connector_space_sz_x = pin_dist*6;
bolt_area_sz_x = bolt_connector_count * bolt_connector_space_sz_x;
bolt_connector_sz_y = 7.5;
bolt_connector_sz_z = 10;
bolt_connector_pin_sz_z = 4.5;

proto_pcb_sz_x = 70;
proto_pcb_sz_y = 50.5;
proto_pcb_sz_z = 1.2;

bolt_connector_pcb_x = proto_pcb_sz_x;
bolt_connector_pcb_y = proto_pcb_sz_y;
bolt_connector_pcb_z = 1.2;

au_connector_d = 6;

batt_sz_x = 63;
batt_sz_y = 59;
batt_sz_z = 19;

batt_box_sz_x = batt_sz_x + 5 + 2 * wall;
batt_box_sz_y = batt_sz_y + 2 * wall;
batt_box_sz_z = batt_sz_z + 2 * wall;

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

module ear(base_dist, height, d=3, cone=false) {
	bolt_hole_cone_center(base_dist=base_dist, hole_d=d, hole_wall=3, height=height, hole_height=cone ? height/2 : height);
}

module ear_holes() {
	translate([0, box_sz_y-30, box_sz_z]) {
		for(xpos=[[wall+6, 180], [box_sz_x-6-wall, 0]]) {
			translate([xpos[0], 0, 0]) {
				rotate([0, 0, xpos[1]]) {
					ear(3, 10, cone=true);
				}
			}
		}
	}
	translate([0, 0, 30]) {
		for(xpos=[[wall+6, 180], [box_sz_x-6-wall, 0]]) {
			translate([xpos[0], 0, 0]) {
				rotate([90+xpos[1], 0, xpos[1]]) {
					ear(3, 10, cone=true);
				}
			}
		}
	}
}

module pcb_holder() {
	// rails
	translate([(box_sz_x-pcb_sz_x)/2, wall, 0]) {
		for(xsgn=[0,1]) {
			translate([xsgn*(pcb_sz_x)-pcb_free_side_border, 0, 0]) {
				// bottom part
				cube([2*pcb_free_side_border, box_sz_y-2*wall-lcd_sz_z, pcb_space_under]);
				translate([0, 0, pcb_space_under+pcb_sz_z+gap]) {
					// top part
					cube([2*pcb_free_side_border, box_sz_y-2*wall-lcd_sz_z, wall]);
				}
			}
			translate([xsgn*(pcb_sz_x+pcb_free_side_border)-pcb_free_side_border, 0, pcb_space_under]) {
				// middle part
				cube([pcb_free_side_border, box_sz_y-2*wall-lcd_sz_z, pcb_sz_z+gap]);
			}
		}
	}
	// bottom wall
	cube([box_sz_x, box_sz_y, wall]);
	// side walls
	translate([0, 0, 0]) {
		for(xsgn=[0,1]) {
			translate([xsgn*(box_sz_x-wall), 0, 0]) {
				cube([wall, box_sz_y, box_sz_z]);
				if(xsgn == 0) {
					// ds18b20 connector
					translate([0, 20, box_sz_z-30]) {
						rotate([0, 90, 0]) {
							cylinder(d=au_connector_d, h=30, center=true);
						}
					}
				} else {
					translate([0, 20, box_sz_z-30]) {
						rotate([90, 0, 90]) {
							switch_holes();
						}
					}
				}
			}
		}
	}
	translate([0, box_sz_y-wall, 0]) {
		difference() {
			union() {
				// lcd wall
				cube([box_sz_x, wall, box_sz_z]);
				translate([(box_sz_x-pcb_sz_x)/2, 0, pcb_space_under]) {
					translate([-pcb_lcd_shift_x, 0, pcb_sz_z+6]) {
						rotate([90, 0, 0]) {
							lcd_holes(h=pcb_lcd_stand_sz_y, d=hole_d_through+2*wall);
						}
					}
				}
			}
			translate([(box_sz_x-pcb_sz_x)/2, 0, pcb_space_under]) {
				translate([-pcb_lcd_shift_x, 0, pcb_sz_z+6]) {
					rotate([90, 0, 0]) {
						lcd_all_holes(h=50, d=hole_d_through);
					}
				}
			}
		}
	}
	ear_holes();
}

module pcb_holder_top() {
	difference() {
		union() {
			translate([0, 0, box_sz_z-wall]) {
				// top wall
				cube([box_sz_x, box_sz_y, wall]);
			}
			// connector wall
			translate([0, 0, 0]) {
				cube([box_sz_x, wall, box_sz_z]);
			}
			// connectors wall box
			translate([(box_sz_x-pcb_sz_x)/2+10.7-2*wall, 0, pcb_space_under+pcb_sz_z]) {
				cube([bolt_connector_space_sz_x*3+2*wall, bolt_connector_sz_y+2*wall, box_sz_z-pcb_space_under-pcb_sz_z]);
			}
		}
		translate([(box_sz_x-pcb_sz_x)/2, 0, pcb_space_under]) {
			for(xi=[0, 1, 2]) {
				translate([xi*bolt_connector_space_sz_x+10.7-gap, 0, pcb_sz_z]) {
					cube([bolt_connector_sz_x+2*gap, bolt_connector_sz_y, box_sz_z]);
				}
			}
		}
	}
}

module pcb_subst() {
	translate([(box_sz_x-pcb_sz_x)/2, wall, pcb_space_under]) {
		// pcb
		cube([pcb_sz_x, pcb_sz_y, pcb_sz_z]);
		// connectors
		for(xi=[0, 1, 2]) {
			translate([xi*bolt_connector_space_sz_x+10.7, 0, pcb_sz_z]) {
				cube([bolt_connector_sz_x, bolt_connector_sz_y, bolt_connector_sz_z]);
			}
		}
		translate([-pcb_lcd_shift_x, pcb_sz_y+3.5, pcb_sz_z+6]) {
			cube([lcd_sz_x, lcd_sz_z, lcd_sz_y]);
			rotate([90, 0, 0]) {
				lcd_all_holes(h=lcd_pcb_sz_z, d=hole_d_through);
			}
		}
	}
}

module batt_box() {
	for(xpos=[0,1]) {
		translate([xpos*(batt_box_sz_x-wall), 0, 0]) { 
			cube([wall, batt_box_sz_y, batt_box_sz_z]);
		}
	}
	for(ypos=[0,1]) {
		translate([0, ypos*(batt_box_sz_y-wall), 0]) { 
			cube([batt_box_sz_x, wall, batt_box_sz_z]);
		}
	}
	// bottom wall
	cube([batt_box_sz_x, batt_box_sz_y, wall]);
}

//top_lid();
//bottom_box();

translate([0, 0, 0]) {
//	pcb_holder_top();
}
%pcb_subst();
pcb_holder();
translate([-30, 0, 60]) {
	rotate([0, 90, 0]) {
//		batt_box();
	}
}
