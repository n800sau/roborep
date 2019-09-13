use <../lib/attacher.scad>

opi_width = 56;
opi_length = 85.5;
opi_holes = 2.8;
opi_hole_dist = 3.0;
opi_pcb_height = 1.6;

opi_power_width = 8.2;
opi_power_height = 6.2;
opi_power_depth = 12.7;
opi_power_off_x = 7.5;
opi_power_off_y = -2.0;

opi_hdmi_width = 15.2;
opi_hdmi_height = 5.7;
opi_hdmi_depth = 11.6;
opi_hdmi_off_x = opi_power_off_x + opi_power_width + 14.5;
opi_hdmi_off_y = opi_power_off_y;

opi_audio_diameter = 5.1;
opi_audio_depth = 13.0;
opi_audio_off_x = opi_hdmi_off_x + opi_hdmi_width + 17.5;
opi_audio_off_y = opi_hdmi_off_y;

opi_sd_width = 14.8;
opi_sd_height = 2.0;
opi_sd_depth = 14.8;
opi_sd_off_y = 16.5;

opi_reset_diameter = 4.0;
opi_reset_depth = 5.0;
opi_reset_off_y = 8.5;
opi_reset_off_x = -0.8;
opi_reset_off_z = 1.1;

opi_cam_width = 16.5;
opi_cam_depth = 5.0;
opi_cam_height = 2.0;
opi_cam_off_y = 15.5;
opi_cam_off_x = 1.0;

opi_micro_usb_width = 7.9;
opi_micro_usb_height = 2.9;
opi_micro_usb_depth = 5.8;
opi_micro_usb_off_y = opi_cam_off_y + opi_cam_width + 3.5;
opi_micro_usb_off_x = -0.8;

opi_ir_width = 6.0;
opi_ir_height = 8.3;
opi_ir_depth = 5.5;
opi_ir_off_x = 16.5;
opi_ir_off_y = -0.9;

opi_single_usb_width = 5.8;
opi_single_usb_height = 13.2;
opi_single_usb_depth = 19.7;
opi_single_usb_off_y = 6.8;
opi_single_usb_off_x = -4.3;

opi_lan_width = 16.1;
opi_lan_depth = 21.7;
opi_lan_height = 13.4;
opi_lan_off_y = opi_single_usb_off_y + opi_single_usb_width + 3.0;
opi_lan_off_x = -4.2;

opi_dual_usb_width = 13.2;
opi_dual_usb_depth = 17.5;
opi_dual_usb_height = 15.8;
opi_dual_usb_off_y = opi_lan_off_y + opi_lan_width + 4.3;
opi_dual_usb_off_x = -4.1;

standoff_height = 3.0;
standoff_diameter = 6.0;
standoff_hole = 2.9;

outer_wall_size = 2.0;
bottom_top_size = 2.0;

total_height = (2 * bottom_top_size) + standoff_height + opi_pcb_height + opi_dual_usb_height;

wall_height = total_height - (2 * bottom_top_size);

top_standoff_height = wall_height - standoff_height - opi_pcb_height;
top_standoff_hole = 3.0;

sma_diameter = 5.9;
sma_off_x = 15.0;
sma_off_y = 15.0;

cable_cut_width = 51.0;
cable_cut_depth = 5.4;
cable_cut_off_x = 8.8;
cable_cut_off_y = 1.2;

text_depth = 0.25;

text1_font = "Liberation Sans";
text1_size = 7;
text1 = "OrangePi PC Plus";
text1_off_x = 6;
text1_off_y = 20;

text2_font = "Liberation Sans";
text2_size = 5.5;
text2 = "by xythobuz.de";
text2_off_x = 27;
text2_off_y = 40.5;

corner_helper_size = 20;
corner_helper_height = 0.2;

cutout_gap = 0.2;

$fn = 20;

module orangepi_pc_plus() {
	difference() {
		// pcb
		cube([opi_length, opi_width, opi_pcb_height]);
		
		// mounting holes
		for (i = [0 : 1]) {
			for (j = [0 : 1]) {
				translate([i * opi_length, j * opi_width, -1])
					translate([-((i * 2) - 1) * opi_hole_dist, 0, 0])
					translate([0, -((j * 2) - 1) * opi_hole_dist, 0])
					cylinder(d = opi_holes, h = opi_pcb_height + 2);
			}
		}
	}
	
	// power connector
	translate([opi_power_off_x, opi_power_off_y, opi_pcb_height])
		cube([opi_power_width, opi_power_depth, opi_power_height]);
	
	// hdmi connector
	translate([opi_hdmi_off_x, opi_hdmi_off_y, opi_pcb_height])
		cube([opi_hdmi_width, opi_hdmi_depth, opi_hdmi_height]);
	
	// audio connector
	translate([opi_audio_off_x + (opi_audio_diameter / 2), opi_audio_off_y + opi_audio_depth, opi_pcb_height + (opi_audio_diameter / 2)])
		rotate([90, 0, 0])
		cylinder(d = opi_audio_diameter, h = opi_audio_depth);
	
	// micro sd card slot
	translate([0, opi_sd_off_y, -opi_sd_height])
		cube([opi_sd_depth, opi_sd_width, opi_sd_height]);
	
	// reset button
	translate([opi_reset_off_x, opi_reset_off_y + (opi_reset_diameter / 2), opi_pcb_height + (opi_reset_diameter / 2) + opi_reset_off_z])
		rotate([0, 90, 0])
		cylinder(d = opi_reset_diameter, h = opi_reset_depth);
	
	// camera connector
	translate([opi_cam_off_x, opi_cam_off_y, opi_pcb_height])
		cube([opi_cam_depth, opi_cam_width, opi_cam_height]);
	
	// micro usb connector
	translate([opi_micro_usb_off_x, opi_micro_usb_off_y, opi_pcb_height])
		cube([opi_micro_usb_depth, opi_micro_usb_width, opi_micro_usb_height]);
	
	// ir receiver
	translate([opi_length - opi_ir_off_x - opi_ir_width, opi_width - opi_ir_off_y - opi_ir_depth, opi_pcb_height])
		cube([opi_ir_width, opi_ir_depth, opi_ir_height]);
	
	// single usb port
	translate([opi_length - opi_single_usb_off_x - opi_single_usb_depth, opi_width - opi_single_usb_off_y - opi_single_usb_width, opi_pcb_height])
		cube([opi_single_usb_depth, opi_single_usb_width, opi_single_usb_height]);
	
	// ethernet port
	translate([opi_length - opi_lan_off_x - opi_lan_depth, opi_width - opi_lan_off_y - opi_lan_width, opi_pcb_height])
		cube([opi_lan_depth, opi_lan_width, opi_lan_height]);
	
	// dual usb ports
	translate([opi_length - opi_dual_usb_off_x - opi_dual_usb_depth, opi_width - opi_dual_usb_off_y - opi_dual_usb_width, opi_pcb_height])
		cube([opi_dual_usb_depth, opi_dual_usb_width, opi_dual_usb_height]);
}

module case_bottom() {
	// bottom plate
	cube([opi_length + (2 * outer_wall_size), opi_width + (2 * outer_wall_size), bottom_top_size]);


	for(xpos=[-1,1]) {
		translate([opi_length/2+20*xpos, opi_width/2, 0]) {
			rotate([0, 0, 90]) {
				attacher(opi_width+20, wall=bottom_top_size);
			}
		}
	}

	// print helper corner disks
//	for (i = [0 : 1]) {
//		for (j = [0 : 1]) {
//			translate([i * (opi_length + (2 * outer_wall_size)), j * (opi_width + (2 * outer_wall_size)), 0])
//			cylinder(d = corner_helper_size, h = corner_helper_height);
//		}
//	}
	
	// standoffs
	translate([outer_wall_size, outer_wall_size, 0])
	for (i = [0 : 1]) {
		for (j = [0 : 1]) {
			translate([i * opi_length, j * opi_width, bottom_top_size])
				translate([-((i * 2) - 1) * opi_hole_dist, 0, 0])
				translate([0, -((j * 2) - 1) * opi_hole_dist, 0])
				difference() {
					cylinder(d = standoff_diameter, h = standoff_height);
					cylinder(d = standoff_hole, h = standoff_height + 1);
				}
		}
	}
	
	// left (micro sd) wall
	translate([0, 0, bottom_top_size])
	difference() {
		cube([outer_wall_size, opi_width + (2 * outer_wall_size), standoff_height + opi_pcb_height]);
		
		// sd card cutout
		translate([-1, outer_wall_size + opi_sd_off_y - cutout_gap, standoff_height - opi_sd_height])
			cube([outer_wall_size + 2, opi_sd_width + (2 * cutout_gap), opi_sd_height + standoff_height]);
	}
	
	// front wall
	translate([outer_wall_size, 0, bottom_top_size])
		cube([opi_length, outer_wall_size, standoff_height + opi_pcb_height]);
	
	// back wall
	translate([outer_wall_size, opi_width + outer_wall_size, bottom_top_size])
		cube([opi_length, outer_wall_size, standoff_height + opi_pcb_height]);
	
	// right wall
	translate([outer_wall_size + opi_length, 0, bottom_top_size])
		cube([outer_wall_size, opi_width + (2 * outer_wall_size), standoff_height + opi_pcb_height]);
}

module case_top() {
	difference() {
		// top plate
		cube([opi_length + (2 * outer_wall_size), opi_width + (2 * outer_wall_size), bottom_top_size]);
		
		// sma connector
		translate([outer_wall_size + sma_off_x, outer_wall_size + opi_width - sma_off_y, -1])
			cylinder(d = sma_diameter, h = bottom_top_size + 2);
		
		// holes in top plate
		translate([outer_wall_size, outer_wall_size, 0])
		for (i = [0 : 1]) {
			for (j = [0 : 1]) {
				translate([i * opi_length, j * opi_width, -1])
					translate([-((i * 2) - 1) * opi_hole_dist, 0, 0])
					translate([0, -((j * 2) - 1) * opi_hole_dist, 0])
					cylinder(d = top_standoff_hole, h = bottom_top_size + 2);
			}
		}
		
		// cable cutout
		translate([outer_wall_size + cable_cut_off_x, outer_wall_size + opi_width - cable_cut_off_y - cable_cut_depth, -1])
			cube([cable_cut_width, cable_cut_depth, bottom_top_size + 2]);
		
		// text in top plate
		translate([text1_off_x, text1_off_y, bottom_top_size - text_depth])
			linear_extrude(height = text_depth + 0.1) {
			text(text1, font = text1_font, size = text1_size);
		}
		
		translate([text2_off_x, text2_off_y, bottom_top_size - text_depth])
			linear_extrude(height = text_depth + 0.1) {
			text(text2, font = text2_font, size = text2_size);
		}
	}
	
	// print helper corner disks
//	for (i = [0 : 1]) {
//		for (j = [0 : 1]) {
//			translate([i * (opi_length + (2 * outer_wall_size)), j * (opi_width + (2 * outer_wall_size)), bottom_top_size - corner_helper_height])
//			cylinder(d = corner_helper_size, h = corner_helper_height);
//		}
//	}
	
	// standoffs
	translate([outer_wall_size, outer_wall_size, 0])
	for (i = [0 : 1]) {
		for (j = [0 : 1]) {
			translate([i * opi_length, j * opi_width, -top_standoff_height])
				translate([-((i * 2) - 1) * opi_hole_dist, 0, 0])
				translate([0, -((j * 2) - 1) * opi_hole_dist, 0])
				difference() {
					cylinder(d = standoff_diameter, h = top_standoff_height);
					
					translate([0, 0, -1])
						cylinder(d = top_standoff_hole, h = top_standoff_height + 2);
				}
		}
	}
	
	// front wall
	translate([outer_wall_size, 0, -top_standoff_height])
	difference() {
		cube([opi_length, outer_wall_size, top_standoff_height]);
		
		// power connector
		translate([opi_power_off_x - cutout_gap, -1, -1])
			cube([opi_power_width + (2 * cutout_gap), outer_wall_size + 2, opi_power_height + 1]);
		
		// hdmi connector
		translate([opi_hdmi_off_x - cutout_gap, -1, -1])
			cube([opi_hdmi_width + (2 * cutout_gap), outer_wall_size + 2, opi_hdmi_height + 1]);
		
		// audio connector
		translate([opi_audio_off_x - cutout_gap, -1, -1])
			cube([opi_audio_diameter + (2 * cutout_gap), outer_wall_size + 2, opi_audio_diameter + 1]);
	}
	
	// back wall
	translate([outer_wall_size, opi_width + outer_wall_size, -top_standoff_height])
	difference() {
		cube([opi_length, outer_wall_size, top_standoff_height]);
		
		translate([opi_length - opi_ir_off_x - opi_ir_width - cutout_gap, -1, -1])
			cube([opi_ir_width + (2 * cutout_gap), outer_wall_size + 2, opi_ir_height + 1]);
	}
	
	// left wall
	translate([0, 0, -top_standoff_height])
	difference() {
		cube([outer_wall_size, opi_width + (2 * outer_wall_size), top_standoff_height]);
		
		// reset button
		translate([-1, outer_wall_size + opi_reset_off_y - cutout_gap, -1])
			cube([outer_wall_size + 2, opi_reset_diameter + (2 * cutout_gap), opi_reset_diameter + opi_reset_off_z + 1]);
		
		// camera connector
		translate([-1, outer_wall_size + opi_cam_off_y - cutout_gap, -1])
			cube([outer_wall_size + 2, opi_cam_width + (2 * cutout_gap), opi_cam_height + 1]);
		
		// micro usb connector
		translate([-1, outer_wall_size + opi_micro_usb_off_y - cutout_gap, -1])
			cube([outer_wall_size + 2, opi_micro_usb_width + (2 * cutout_gap), opi_micro_usb_height + 1]);
	}
	
	// right wall
	translate([outer_wall_size + opi_length, 0, -top_standoff_height])
	difference() {
		cube([outer_wall_size, opi_width + (2 * outer_wall_size), top_standoff_height]);
		
		// single usb port
		translate([-1, outer_wall_size + opi_width - opi_single_usb_off_y - opi_single_usb_width - cutout_gap, -1])
			cube([outer_wall_size + 2, opi_single_usb_width + (2 * cutout_gap), opi_single_usb_height + 1]);
		
		// ethernet port
		translate([-1, outer_wall_size + opi_width - opi_lan_off_y - opi_lan_width - cutout_gap, -1])
			cube([outer_wall_size + 2, opi_lan_width + (2 * cutout_gap), opi_lan_height + 1]);
		
		// dual usb ports
		translate([-1, outer_wall_size + opi_width - opi_dual_usb_off_y - opi_dual_usb_width - cutout_gap, -1])
			cube([outer_wall_size + 2, opi_dual_usb_width + (2 * cutout_gap), opi_dual_usb_height + 1]);
	}
}

module case_whole() {
	case_bottom();
	
	translate([0, 0, total_height - bottom_top_size])
		case_top();
}

// ---------------------
// For looking at it in the OpenSCAD editor:

case_whole();
// place orangepi inside case, for visualization
%translate([outer_wall_size, outer_wall_size, bottom_top_size + standoff_height])
	orangepi_pc_plus();

// ---------------------
// For printing:
/*
case_bottom();
*//*
translate([0, 2 * (opi_width + (2 * outer_wall_size)) + 25, bottom_top_size])
	rotate([180, 0, 0])
	case_top();
*/
// ---------------------
