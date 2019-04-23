// wire set type: "wire" or "carbon"
wire_set_holder_type = "carbon";
//wire_set_holder_type = "wire";

height = 20;

glass_x = 100.8;
glass_y = 82;
glass_h = 2.7;
wall = 3;

pad_y = 2;
wet_x = glass_x + 20;
wet_y = glass_y + 10 + pad_y*2;

wire_holder_z = 5;
wire_holder_h_offset=-6;
wire_hole_d = 2;

istep = 10;

//for comb (and bariers) places
comb_place_top_h = wall+glass_h+15;
comb_place_h = 5;
comb_place_sz_x = 6;
comb_places = 9;
comb_start = -comb_places/2;
comb_end = comb_places/2;

chamber_h = comb_place_top_h;

pad = 0.5;
electrode_plate_x = 5;
electrode_top_x = 6;
electrode_plate_y = wet_y-pad*2;
electrode_plate_bottom = 5;

border_x = comb_place_sz_x-pad*2; // limited by gel box
border_y = wall + 3;
bolt_plate_x = 10;
bolt_plate_y = wall;
bolt_plate_z = 10;
bolt_plate_case_base_y = 30+wall*2;

// for comb
comb_sz_x = comb_place_sz_x - 0.8;
comb_h = comb_place_h;
glass_tooth_gap = 1.5;
comb_tooth_h = comb_place_top_h-comb_place_h-wall-glass_h-glass_tooth_gap;
comb_well_sz_y = 5;
comb_well_sz_x = 2;
comb_handle_h = 3;
comb_handle_sz_x = comb_sz_x - 1;

barrier_h = comb_place_top_h-wall-glass_h+pad;
barrier_thin_h = 10.5;
barrier_thin_sz_x = 2.8;
barrier_sz_x = comb_place_sz_x - 0.8;

leg_h = 40;
leg_sz_x = 20;
leg_h_extra = 3;

hole_d = 3.1;
hole_through_d = 4;

lenc_h = 30;
lenc_sz_x = glass_x;
lenc_sz_y = glass_y;
lenc_led_hole = 20;
lenc_led_hole_dist = 32;

hole_x_pos_list = [17.5];

carbon_h = 12;
// carbon_wire_hole_d = 1.6 // for steel
carbon_wire_hole_d = 1.1;

carbon_d = carbon_wire_hole_d + 3;
carbon_sz_x = 2;
carbox_sz_y = glass_y-pad*2;
//carbox_sz_y = 10;

power_pcb_sz_x = 48.5;
power_pcb_sz_y = 1.2;
power_pcb_h = 24.2;

stand_sz = 12;
