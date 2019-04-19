$fn = 50;

wall = 3;

block_sz_x = 40.2;
block_sz_y = 40.2;
block_sz_z = 4.2;

hole_dist = 72;

cover_sz_x = 80;
cover_sz_y = 80;

hole_d = 4;

heat_block_sz_x = 20;
heat_block_sz_y = 16;
heat_block_sz_z = 12;

heat_block_cover_off_z = heat_block_sz_z - block_sz_z;

module spec_cross(make_hole=true) {
  difference() {
    translate([0, 0, -block_sz_z/2]) {
      difference() {
        union() {
          cube([block_sz_x+14, block_sz_y+14, wall], center=true);
          for(angl=[0,1]) {
              rotate([0, 0, 45+angl*90]) {
                cube([cover_sz_x+20, 15, wall], center=true);
                for(xsgn=[-1, 1]) {
                  translate([xsgn*(cover_sz_x+20)/2, 0, 0]) {
                    cylinder(d=15, h=wall, center=true);
                  }
                }
              }
          }
        }
        if(make_hole) {
          cube([block_sz_x-wall, block_sz_y-wall, wall], center=true);
        } else {
          for(angle=[0, 90, 180, 270]) {
            rotate([0, 0, angle]) {
              translate([(block_sz_x-wall)/2-4, (block_sz_y-wall)/2-7.5, 0]) {
                cylinder(d=8, h=wall, center=true);
              }
              translate([(block_sz_x-wall)/2-17, (block_sz_y-wall)/2-7.5, 0]) {
                cylinder(d=8, h=wall, center=true);
              }
            }
          }
        }
        for(xsgn=[-1,1]) {
          for(ysgn=[-1,1]) {
            translate([xsgn*hole_dist/2, ysgn*hole_dist/2, 0]) {
              cylinder(d=hole_d, h=50, center=true);
            }
          }
        }
      }
    }
  }
}

module peltier_fan_holder() {
  difference() {
    cube([block_sz_x+2*wall, block_sz_y+2*wall, block_sz_z], center=true);
    cube([block_sz_x, block_sz_y, block_sz_z], center=true);
    for(xsgn=[-1,1]) {
      for(ysgn=[-1,1]) {
        translate([xsgn*block_sz_x/2, ysgn*block_sz_y/2, 0]) {
          cube([10, 10, 10], center=true);
        }
      }
    }
  }
  spec_cross();
}

module heater_holder() {
  spec_cross(false);
}

module stand() {
  for(xsgn=[-1,1]) {
    for(ysgn=[-1,1]) {
      translate([xsgn*hole_dist/2, ysgn*hole_dist/2, 0]) {
        difference() {
          cylinder(d=hole_d+wall*2, h=heat_block_cover_off_z, center=true);
          cylinder(d=hole_d, h=heat_block_cover_off_z, center=true);
        }
      }
    }
  }
}
 
rotate([180, 0, 0]) {
  translate([0, 0, (block_sz_z+wall)/2]) {
    peltier_fan_holder();
  }
}
translate([0, 0, heat_block_cover_off_z+(block_sz_z+wall)/2]) {
  heater_holder();
}
translate([0, 0, (block_sz_z+wall)/2]) {
  stand();
}
