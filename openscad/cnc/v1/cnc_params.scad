$fn = 40;

wall = 3;

rod_d = 12;
rod_sz = 150;
rod_bearing_d = 21;
rod_bearing_sz = 30;
rod_rod_dist = 30+rod_bearing_d;
rod_bearing_pos_z = [15+12/2, 15 + 12/2 + rod_rod_dist + 12/2];
lead_screw_d = 8;
lead_screw_bearing_d = 22;
lead_screw_bearing_sz = 7;
lead_screw_nut_d = 11;
lead_screw_nut_pos_z = rod_bearing_pos_z[0] + 18.5 + 8/2 + 12/2;
lead_screw_nut_hole_d = 2.4;
lead_screw_nut_hole_r_dist = 8;

echo("rod 1-lead ", lead_screw_nut_pos_z-rod_bearing_pos_z[0], ", lead to  rod2 ", lead_screw_nut_pos_z - rod_bearing_pos_z[1]);

base_sz_x = rod_bearing_sz * 2 + 2 * wall;
base_sz_y = rod_bearing_d + 6; // shortest
base_sz_z = rod_sz+wall*2;
hole_d = 4;
hole_d_through = 4.8;
mounting_hole_offset = 5;

motor_sz = 43;
motor_d = 21;
motor_hole_dist = 31;
rod_distance = 67.2 - rod_bearing_d;

top_block_sz_x = base_sz_x + 2 * wall;
top_block_sz_y = motor_sz + 4 * wall + base_sz_y;
top_block_sz_z = 10;

bottom_block_sz_x = top_block_sz_x;
bottom_block_sz_y = top_block_sz_y - 10;
bottom_block_sz_z = lead_screw_bearing_sz + wall;


