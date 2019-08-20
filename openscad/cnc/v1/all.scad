include<cnc_params.scad>
use<gantry_base.scad>
use<gantry.scad>
use<motor_side.scad>
use<bottom_side.scad>

translate([0, -(top_block_sz_y-base_sz_y)/2+wall, (base_sz_z-top_block_sz_z)/2]) {
//	motor_raise();
	motor_side();
}

color("blue")
translate([0, -(top_block_sz_y)/2, 0]) {
  gantry();
}
//gantry_base();
translate([0, -(bottom_block_sz_y-base_sz_y)/2+wall, -(base_sz_z-bottom_block_sz_z)/2]) {
//	bottom_side();
}
