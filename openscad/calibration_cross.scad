//X scale 100/101.5=0.9852 = 98.52%
//Y scale 100/99.3=1.007 = 100.7%

cube([100, 5, 2], center=true);
rotate([0, 0, 90]) {
  cube([100, 5, 2], center=true);
  translate([20, 0, 0]) {
    cube([5, 8, 2], center=true);
  }
}
translate([0, 0, 10+1]) {
//  cube([5, 5, 20], center=true);
}

