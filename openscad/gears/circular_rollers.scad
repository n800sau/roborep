$fn = 50;
ring_h = 2;
ring_w = 3;
inner_d = 44.5;
outer_d = inner_d + ring_w * 2;
roller_r = 5;


difference() {
  cylinder(d=outer_d, h=ring_h, center=true);
  cylinder(d=inner_d, h=ring_h, center=true);
}

step = 360/6;
for(i=[0:step:360]) {
  rotate([0, 0, i]) {
    translate([0, (outer_d-ring_w)/2, 0]) {
      difference() {
        rotate([90, 0, 0]) {
          cylinder(r=roller_r, h=ring_w/2, center=true);
        }
        translate([0, 0, -roller_r/2]) {
          cube([roller_r*2, ring_w/2, roller_r], center=true);
        }
      }
    }
  }
}
