$fn = 40;

wall = 3;

rod_d = 12;
rod_sz = 150;
rod_bearing_d = 16; //?
rod_bearing_sz = 30; //?
rod_bearing_pos_z = [20, 50];
lead_screw_d = 8;
lead_screw_bearing_d = 15; //?
lead_screw_nut_d = 10; //?
lead_screw_nut_pos_z = 35; //?
lead_screw_nut_hole_d = 2.4; //?
lead_screw_nut_hole_r_dist = 4;

base_sz_x = rod_bearing_sz * 2;
base_sz_y = rod_bearing_d + 6; // shortest
base_sz_z = min(rod_sz-10, rod_bearing_pos_z[1] + rod_bearing_d);
hole_d = 4;

motor_sz = 30; //?
motor_d = 15;
motor_hole_dist = 24; //?

top_block_sz_x = base_sz_x + 2 * wall;
top_block_sz_y = motor_sz + 4 * wall + base_sz_y;
top_block_sz_z = 10;

bottom_block_sz_x = top_block_sz_x;
bottom_block_sz_y = top_block_sz_y;
bottom_block_sz_z = 10;

