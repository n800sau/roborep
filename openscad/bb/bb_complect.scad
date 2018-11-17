include <bb_box_params.scad>;
use <bb_anycube_case.scad>
use <bb_anycube_lid.scad>

translate([ll_thickness+1, 0, 30]) {
  rotate([0, 180, 0]) {
    BBLid(show_lid=false, show_attach=true, single_attach=true);
  }
}
//Case();
