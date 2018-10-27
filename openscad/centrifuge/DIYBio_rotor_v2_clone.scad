// This is an OpenSCAD clone of F.Lab's DIYBio rotor v2.
//
// Thomas Kircher <tkircher@gnu.org>, 8/11/2017

numbers = ["1", "2", "3", "4", "5", "6"];

base_d1 = 85;
base_d2 = 50;
base_bottom_h = 16;
base_top_h = 3;
underlay_h = 4;

module main_rotor_body() {
//    base_d1 = 73.2;
//    base_d2 = 57.8;
//    base_bottom_h = 6;
//    base_top_h = 10.2;

  
    // Rotor base
    translate([0, 0, 0])
        cylinder(d = base_d1, h = base_bottom_h, $fn = 180);

      translate([0, 0, -underlay_h])
          cylinder(d = base_d1, h = underlay_h, $fn = 180);
    
    // Rotor upper taper
    translate([0, 0, base_bottom_h])
        cylinder(d1 = base_d1, d2 = base_d2, h = base_top_h, $fn = 280);
}

module rotor_body_cutout() {
//  rotor_d1 = 20;
//  rotor_d2 = 48.5;
//  rotor_h = 14.4;

  rotor_d1 = 30;
  rotor_d2 = 58.5;
  rotor_h = 24.4;
  
  hole_dist = 21;
  hole_d = 3;
  
  
    // Rotor inner taper
    translate([0, 0, 2])
        cylinder(d1 = rotor_d1, d2 = rotor_d2, h = rotor_h, $fn = 220);

    for(i=[0: 6]) {
    // Rotor mount holes
      rotate([0, 0, i*60]) {
        translate([0, hole_dist/2, -underlay_h])
          cylinder(d = hole_d, h = 6.1, $fn = 60);
      }
//      rotate([0, 0, i*60+30]) {
//        translate([0, 35, -underlay_h])
//          cylinder(d = 2.6, h = 20, $fn = 60);
//        translate([0, 25, -underlay_h])
//          cylinder(d = 2.6, h = 20, $fn = 60);
//        translate([5, 30, -underlay_h])
//          cylinder(d = 2.6, h = 20, $fn = 60);
//        translate([-5, 30, -underlay_h])
//          cylinder(d = 2.6, h = 20, $fn = 60);
//      }
    }

    // Numeral cutouts
    for(i = [0:5]) {
        rotate([-10, 0, 60 * i-4])
          translate([0, rotor_d2/2+6, rotor_h/2+9])
            linear_extrude(height = 3)
                rotate([0, 0, 180])
                    text(numbers[i], size = 7, font="Arial:style=Bold");
    }
}

// Eppendorf 1.5mL tubes are 10.7mm OD
module rotor_slot_cutout() {
  //dist = 31;
  angle = 85;
  translate([0, 0, 3])
    for(i = [0:5]) {
        rotate([0, 0, -60 * i])
          translate([0, 95, 0])
            rotate([angle, 0, 0])
                cylinder(d = 12, h = 100, $fn = 80);
    }
}

difference() {
//union() {
    main_rotor_body();

    union() {
        rotor_body_cutout();
        rotor_slot_cutout();
      translate([0, 0, -underlay_h])
          rotate([0, 0, 60]) {
            translate([base_d1/2-11, 0, 2.5])
            cube([6, 6, 5], center=true);
          }
    }
}