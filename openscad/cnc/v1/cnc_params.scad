$fn = 40;

wall = 3;

rod_d = 12;
rod_sz = 152;
rod_bearing_d = 21;
rod_bearing_sz = 30;
rod_rod_dist = 34+rod_bearing_d;
rod_bearing_pos_z = [15+12/2, 15 + 12/2 + rod_rod_dist + 12/2];
lead_screw_d = 8;
lead_screw_bearing_d = 22;
lead_screw_bearing_sz = 7;
lead_screw_nut_d = 10.6;
lead_screw_nut_pos_z = rod_bearing_pos_z[0] + 20.5 + 8/2 + 12/2;
lead_screw_nut_hole_d = 2.4;
lead_screw_nut_hole_r_dist = 8;
lead_screw_nut_attach_plate_d = 22.4;

echo("rod 1-lead ", lead_screw_nut_pos_z-rod_bearing_pos_z[0], ", lead to  rod2 ", lead_screw_nut_pos_z - rod_bearing_pos_z[1]);

base_sz_x = rod_bearing_sz * 2 + 2 * wall;
base_sz_y = rod_bearing_d + 6; // shortest
base_sz_z = rod_sz+wall*2;

//28 65
echo("base x: ", base_sz_x, ", base y:", base_sz_y);

hole_d = 3.8;
hole_d_through = 4.8;
mounting_hole_offset = 5;

motor_sz = 43;
motor_d = 24;
motor_hole_dist = 30.2;
rod_distance = 66.2 - rod_bearing_d; //rod_distance/2 = 23.25??? (66.2 - 21)/2 = 45.2/2 = 22.6 !!!
// motor shift from rod axis
motor_shift_y = 5;

top_block_sz_x = base_sz_x + 2 * wall;
top_block_sz_y = motor_sz + 4 * wall + base_sz_y;
top_block_sz_z = 10;

bottom_block_sz_x = top_block_sz_x;
bottom_block_sz_y = top_block_sz_y - 10;
bottom_block_sz_z = lead_screw_bearing_sz + wall;

// ganztry_z params

gantry_sz_x = base_sz_x;
gantry_sz_y = rod_bearing_d + 2 * wall;
gantry_sz_z = rod_bearing_sz * 2 + 4;

attach_sz_x = 4;
attach_sz_y = 25;
attach_sz_z = gantry_sz_z-10;

extract_sz_x = lead_screw_nut_attach_plate_d + 2 * wall;
extract_sz_y = extract_sz_x;
extract_sz_z = gantry_sz_z - 15;

rod_dist_x = 67.4 - rod_bearing_d;

attach_side_gap = 0.5;

connector_base_sz_y = 12;

connector_to_work_surface_sz_z = 56;

//////////////////////// duratech drill maket
top_d = 23;
top_sz_z = 37;
above_widest_level_d = 36;
above_widest_level_sz_z = 50;
widest_level_d = 37;
widest_level_sz_z = 7.8;

above_bottom_box_sz_x = 17;
above_bottom_box_sz_y = 11;
above_bottom_box_sz_z = 14;

above_bottom_d = 29;
above_bottom_sz_z = 9.5;
bottom_d = 22.5;
bottom_sz_z = 7.5;

metal_thing_d  = 10.2;
metal_thing_sz_z = 11;
metal_cone_bottom_d = 7.4;
metal_cone_bottom_sz_z = 3;

//////////////////////////

