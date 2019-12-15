include<cnc_params.scad>
use<x_sliding_block.scad>
use<z_sliding_block.scad>
use<motor_side.scad>
use<bottom_side.scad>
use<pen_holder.scad>

translate([0, -(top_block_sz_y-base_sz_y)/2+wall, (base_sz_z-top_block_sz_z)/2]) {
//	motor_raise();
//	motor_side();
}

translate([0, -(top_block_sz_y)/2, 0]) {
color("blue")
	slider_z();
	pen_holder();
}
//slider_x();
translate([0, -(bottom_block_sz_y-base_sz_y)/2+wall, -(base_sz_z-bottom_block_sz_z)/2]) {
//	bottom_side();
}
