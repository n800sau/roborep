h = 5.5;

pad = 0.5;

max_d = 27;
intr_top_d = 14 + pad;
top_h = 3.7;
intr_bottom_d = 16 + pad;
bottom_h = 1;
extr_d = min(max_d, intr_bottom_d + 6);
bolt_d = 4 + pad;
bolt_cap_d = 7 + pad;
full_h = bottom_h + top_h + h;

angle_step = 360/6;
cone_d1 = 1;
cone_d2 = 2;
cone_h = 6 ;

$fn = 50;

module round_thing() {
  difference() {
    cylinder(d=extr_d, h=full_h);
    cylinder(d=intr_bottom_d, h=bottom_h);
    cylinder(d=intr_top_d, h=bottom_h+top_h);
    cylinder(d=bolt_d, h=full_h);
  }
  for(i=[0:angle_step:360]) {
    rotate([0, 0, i]) {
      translate([4.5, 0, bottom_h+top_h-cone_h]) {
        cylinder(d1=cone_d1, d2=cone_d2, h=cone_h);
      }
    }
  }
}

round_thing();
