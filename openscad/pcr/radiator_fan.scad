wall = 2;
$fn = 50;

difference() {
  minkowski()
  {
    cube([42, 38, wall/2], center=true);
    cylinder(r=5, h=wall/2, center=true);
  }
  cylinder(d=42, h=wall, center=true);
  for(ypos=[-15.75, +15.75]) {
    for(xpos=[-21, 21]) {
      translate([xpos, ypos, 0]) {
        cylinder(d=3.7, h=wall, center=true);
      }
    }
  }
}
translate([wall, 0, -wall/2-22]) {
  difference() {
    cube([18.5+wall*2, 44+wall*2, 46+wall], center=true);
    cube([18.5, 44, 46+wall], center=true);
    cube([18.5+wall*2, 24, 46+wall], center=true);
    translate([wall, 0, 0]) {
      cube([18.5+wall*2, 38, 46+wall], center=true);
    }
    for(zpos=[-14, 14]) {
      for(ypos=[-18, 18]) {
        translate([-10, ypos, zpos]) {
          rotate([0, 90, 0]) {
            cylinder(d=4, h=10, center=true);
          }
        }
      }
    }
  }
}

