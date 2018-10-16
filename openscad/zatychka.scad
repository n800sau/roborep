echo(version=version());

main_radius = 11;
int_height = 11.5;
$fn = 50;

  difference() {
    union() {

    color("blue")
      translate([0, 0, 0.5])
        cylinder(h = 1.5, r=main_radius + 4.5, center=true);


      color("red")
        translate([0, 0, 0.5]) {
          difference() {
            cylinder(h = int_height+1.5, r=main_radius);
            cylinder(h = int_height+1.5, r=main_radius-2);
          }
        }

      color("green")
        translate([0, 0, int_height+1]) {
          rotate_extrude() {
            translate([main_radius, 0, 0]) {
              circle(r=1);
            }
          }
        }
    }
    
    color("orange") {
      translate([0, 0, int_height/2+3]) {
        cube([50, 8, int_height+3], center=true);
      }
    }
  }
