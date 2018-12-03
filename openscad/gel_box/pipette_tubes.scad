mk5_d = 4.5;
mk5_h = 31;
mk200_d = 6.5;
mk200_h = 50;
big_tubes_d = 11.5;
big_tubes_h = 75;

module holes(d, h=50) {
  for(x=[-3:3]) {
    for(y=[-2:2]) {
      translate([x*d*2, y*d*2, 0]) {
        cylinder(d=d, h=h, center=true);
      }
    }
  }
}

//union() {
difference() {
  cube([70, 60, 2], center=true);
  holes(d=mk5_d);
}

//translate([100, 0, 0]) {
//  holes(d=mk200_d);
//}
