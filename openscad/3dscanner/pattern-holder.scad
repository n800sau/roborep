flaw = 1.6;

cube([150, 30, 3], center=true);

translate([0, 0, 3]) {
  
  translate([0, 1.5+flaw/2, 0]) {
    cube([150, 3, 5], center=true);
    translate([0, 0, 50]) {
      translate([-75+2.5, 0, 0]) {
        cube([5, 3, 100], center=true);
      }
      translate([75-2.5, 0, 0]) {
        cube([5, 3, 100], center=true);
      }
    }
  }
  translate([0, -1.5-flaw/2, 0]) {
    cube([150, 3, 5], center=true);
    translate([0, 0, 50]) {
      translate([-75+2.5, 0, 0]) {
        cube([5, 3, 100], center=true);
      }
      translate([75-2.5, 0, 0]) {
        cube([5, 3, 100], center=true);
      }
    }
  }
  
}
