//use <spiral.scad>
use <threads.scad>


wall = 1.5;

tube_top_d = 9.6;
tube_top_h = 17;
tube_middle_d = 7.5;
tube_middle_h = 12.1;
tube_bottom_d = 4;

top_h = 3;
bottom_h = 3;

container_d = tube_top_d + 2 * wall;
container_h = tube_top_h+tube_middle_h-bottom_h;

bottom_d = container_d + 2 * wall;

top_d = container_d + 2 * wall;

tsens_h = 2.4;
tsens_d = 2.4;
tsens_leg_d = 1.2;

ear_hole_d = 3.6;
ear_sz = ear_hole_d + 2 * wall;

$fn = 50;

module pcr_tube() {
  translate([0, 0, tube_middle_h]) {
    cylinder(d2=tube_top_d, d1=tube_middle_d, h=tube_top_h);
  }
  cylinder(d2=tube_middle_d, d1=tube_bottom_d, h=tube_middle_h);
}

module tube_container() {
  translate([0, 0, container_h+bottom_h-top_h]) {
    cylinder(d=top_d, h=top_h);
  }
  translate([0, 0, bottom_h]) {
    union() {
//    difference() {
      cylinder(d=container_d, h=container_h);
      translate([0, 0, 0]) {
//      ball_groove(10, container_h-7, container_d, 1);
//spiralSimple(height=30,Radius=10,baseRadius=1,frequency=10,resolution=100);
//hex_screw(thread_outer_diameter,thread_step,
//step_shape_degrees,thread_length,resolution,
//countersink,head_diameter,0,non_thread_length,non_thread_d

//spiralSimple(height=container_h-5,Radius=container_d/2,baseRadius=1,frequency=6000,resolution=1000);
  metric_thread(container_d+1, pitch=1.5, length=container_h, right_handed=false);


//socket_screw(container_d, 2, 40, container_h, 5, 2, 0, 0, 0, 0);

//      hex_nut(container_d, container_h, 2, 50, 10, 5);

//      hex_screw(container_d, 2, 80, container_h, 5, 0, 0, 0, 0, 0);
//      cylinder(d=container_d-10, h=container_h);
      }


    }
  }
  difference() {
    union() {
      cylinder(d=bottom_d, h=bottom_h);
      translate([0, 0, bottom_h/2]) {
        cube([bottom_d+ear_sz, ear_sz, bottom_h], center=true);
        for(sgn=[-1,1]) {
          translate([sgn*(bottom_d+ear_sz)/2, 0, 0]) {
            cylinder(d=ear_sz, h=bottom_h, center=true);
          }
        }
      }
    }
    for(sgn=[-1,1]) {
      translate([sgn*(bottom_d+ear_sz)/2, 0, bottom_h/2]) {
          cylinder(d=ear_hole_d, h=bottom_h, center=true);
        }
    }
//    translate([0, 0, bottom_h-tsens_h]) {
//      cylinder(d=tsens_d, h=tsens_h);
//    }
  }
}

difference() {
  tube_container();
  pcr_tube();
}
