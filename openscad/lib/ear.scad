$fn = 30;

module bolt_hole_cone(
    base_height=3,
    hole_d=2.9,
    wall = 2,
    wall_extra = 3,
    height_extra = 7,
    border = 0.1
) {

  out_d = hole_d + wall * 2;
  
  translate([-out_d/2 - wall_extra, 0, -base_height]) {
    difference() {
      hull() {
        cylinder(d=out_d, h=base_height);
        translate([0, -out_d/2, 0]) {
          cube([out_d/2 + wall_extra, out_d, base_height]);
        }
        translate([out_d/2 + wall_extra-border, -out_d/2, -height_extra]) {
          cube([border, out_d, 1]);
        }
      }
      translate([0, 0, -height_extra]) {
        cylinder(d=hole_d, h=base_height+height_extra);
      }
    }
  }
  
}

module bolt_hole_cone_center(
    hole_height=4,
    hole_d=2.9,
    hole_wall = 2,
    base_dist = 3,
    height = 7,
    border = 0.1
) {

  out_d = hole_d + hole_wall * 2;
  translate([0, 0, -hole_height/2]) {
    difference() {
      hull() {
        cylinder(d=out_d, h=hole_height, center=true);
        translate([(out_d/2+base_dist)/2, 0, 0]) {
          cube([out_d/2+base_dist, out_d, hole_height], center=true);
          translate([(out_d/2+base_dist-border)/2, 0, -hole_height/2-(height-hole_height)+1/2]) {
            cube([border, out_d, 1], center=true);
          }
        }
      }
      cylinder(d=hole_d, h=hole_height+2*height, center=true);
    }
  }
  
}

translate([3+2+2.9+01, 0, 0]) {
  %cube([5, 10, 20], center=true);
  translate([-7.5, 0, 2.5]) {
    %cube([10, 10, 5], center=true);
  }
}
bolt_hole_cone_center();
