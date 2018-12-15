use <led_pcb_holder.scad>
use <pcb_led_cvf.scad>

test_mode = false;

$fn = 50;

int_w = 80;
int_l = 94;

wall = 2;

ext_w = 88;
ext_l = 104;

paper_z = 0.5;

z = 4 + paper_z;

pcb_wall_z = 30 + wall*2;

attach_l = 4;
attach_w = wall;
attach_p_l = 10;
attach_p_z = 4;
attach_hole_d = 1.9;
attach_hole_through_d = 2.5;

module switch_holes() {
  cube([9.5, 50, 5], center=true);
  translate([9.5, 0, 0]) {
    rotate([90, 0, 0]) {
      cylinder(d=2.5, h = 50, center=true);
    }
  }
  translate([-9.5, 0, 0]) {
    rotate([90, 0, 0]) {
      cylinder(d=2.5, h = 50, center=true);
    }
  }
}

module power_side_attachment() {
  translate([0, attach_w-ext_w/2, 0.5]) {
    cube([ext_l, attach_w, attach_p_z+1], center=true);
  }
  translate([attach_l/2-ext_l/2, attach_w-ext_w/2, pcb_wall_z/2-wall/2]) {
    cube([attach_l, attach_w, pcb_wall_z], center=true);
    translate([(attach_p_l-attach_l)/2, 0, (pcb_wall_z-attach_p_z)/2]) {
      cube([attach_p_l, attach_w, attach_p_z], center=true);
    }
  }
  translate([ext_l/2-attach_l/2, attach_w-ext_w/2, pcb_wall_z/2-wall/2]) {
    cube([attach_l, attach_w, pcb_wall_z], center=true);
    translate([(attach_l-attach_p_l)/2, 0, (pcb_wall_z-attach_p_z)/2]) {
      cube([attach_p_l, attach_w, attach_p_z], center=true);
    }
  }
}

module paper_holder() {
  difference() {
    union() {
      difference() {
        cube([ext_l, ext_w, z], center=true);
        cube([int_l, ext_w, z], center=true);
        cube([ext_l, ext_w-4, paper_z], center=true);
      }
      power_side_attachment();
    }
    attach_holes(through=false);
  }
}

module box4light() {
  difference() {
    union() {
      translate([0, 0, -wall/2]) {
        cube([ext_l, ext_w-wall, wall], center=true);
      }
      for(l_pos=[-ext_l/2+wall/2,ext_l/2-wall/2]) {
        translate([l_pos, 0, pcb_wall_z/2]) {
          cube([wall, ext_w-wall, pcb_wall_z], center=true);
        }
      }
      power_side_attachment();
    }
    attach_holes(through=false);
  }
}

module attach_holes(through=true) {
  d = through ? attach_hole_through_d : attach_hole_d;
  for(p=[
      [-int_w/2, 1],
      [int_w/2, 1],
      [0, 1],
      [-int_w/2-5, pcb_wall_z-(attach_p_z+wall)/2],
      [int_w/2+5, pcb_wall_z-(attach_p_z+wall)/2],
      ]) {
    translate([p[0], 0, p[1]]) {
      rotate([90, 0, 0]) {
        cylinder(d=d, h=ext_w*2, center=true);
      }
    }
  }
  for(p=[5,pcb_wall_z-10]) {
    translate([0, ext_l/2-14, p]) {
      rotate([0, 90, 0]) {
        cylinder(d=d, h=ext_l*2, center=true);
      }
    }
  }
}

module power_side() {
  difference() {
    translate([0, -ext_w/2, (pcb_wall_z-wall)/2]) {
      cube([ext_l, wall, pcb_wall_z+wall], center = true);
    }
    // holes for led holder
    translate([16, 0, 0]) {
      a_hole();
    }
    translate([-16, 0, 0]) {
      a_hole();
    }
    // holes for led power pcb
    translate([29, 0, -1]) {
      a_hole();
    }
    translate([-29.5, 0, -1]) {
      a_hole();
    }
    // holes for wires
    translate([-14, 0, -6]) {
      a_hole(4);
    }
    translate([14, 0, 5]) {
      a_hole(4);
    }
    // wires for switch
    translate([30, 0, 7]) {
      a_hole(4);
    }
    translate([30, 0, -9]) {
      a_hole(4);
    }
    // wires for socket (
    translate([-30, 0, 7]) {
      a_hole(4);
    }
    translate([-30, 0, -9]) {
      a_hole(4);
    }
    // power hole
    translate([-42, 0, 0]) {
      a_hole(d=8.8);
    }
    // switch
    translate([42, -40, 15]) {
      rotate([0, 90, 0]) {
        switch_holes();
      }
    }
    attach_holes(through=true);
  }
}

module a_hole(d=3.2, h=50) {
  translate([0, -ext_w/2, pcb_wall_z/2]) {
    rotate([90, 0, 0]) {
      cylinder(d=d, h=h, center=true);
    }
  }
}

module led_light_box(paper=false) {

  difference() {
  //union() {
    if(paper) {
      paper_holder();
    } else {
      box4light();
    }
  }
  
  if(test_mode) {
    color("blue")
    translate([0, -int_w/2, pcb_wall_z/2]) {
      rotate([-90, -30, 0]) {
        led_pcb_holder();
      }
    }
    
    color("green")
    translate([25, -ext_w/2, pcb_wall_z-5]) {
      rotate([90, 180, 0]) {
        pcb_led_cvf();
      }
    }
  }
  
}

led_light_box(paper=false);
//power_side();

