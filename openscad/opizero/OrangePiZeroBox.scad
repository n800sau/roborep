$fa=100;
$fn=100;
delta_d = 0.5;
delta_l = 0.4;


// Размеры платы
w = 46; // Ширина платы ( стороны гребёнок)
l = 48; // Длина платы ( стороны разъёмов)
h_b = 6; // Высота стенок нижней части корпуса
h_t = 6.6; // Высота стенок верхней части корпуса
pcb_thick = 1.8; // Толщина платы
r = 3; // Радиус скругления углов корпуса
r_pcb = 2; // Радиус скругления углов платы

thumb_b_h = 3; // Высота внутренних угловых стоек под плату в нижней части корпуса
thumb_t_h = 7.5; // Высота внутренних угловых стоек под плату в верхней части корпуса

base = 1.5; // Толщина нижней и верхней поверхности
wall = 1.6; // Толщина боковых стенок

// Крепёжные отверстия
d = 3.3; // Диаметр крепёжных отверстий платы
d_shift = 3; // Смещение центра крепёжного отверстия от края платы
cap_d = 5.5 + delta_d; // Диаметр шляпки винта
cap_h = 2; // Высота шляпки винта

// Ethernet
eth_w = 17.2; // Ширина разъёма Ethernet
eth_h = 14; // Высота разъёма Ethernet
eth_shift_w = 14.5; // Смещение окна разъёма Ethernet от правой стороны платы (сторона с двойной гребёнкой)

// USB
usb_w = 7; // Ширина разъёма USB
usb_h = 14; // Высота разъёма USB
usb_shift_w = 33.3; // Смещение окна разъёма USB от правой стороны платы (сторона с двойной гребёнкой)

// microSD
sd_w = 14; // Ширина окна под SD карточку
sd_h = 2.4; // Высота окна под SD карточку
sd_shift_w = 11.5; // Смещение окна SD карточки от правой стороны платы (сторона с двойной гребёнкой)

// microUSB
musb_w = 9; // Ширина окна под разъём microUSB
musb_h = 3.6; // Высота окна под разъём microUSB
musb_shift_w = 31.5; // Смещение окна разъёма microUSB от правой стороны платы (сторона с двойной гребёнкой)

// Радиатор процессора
cpu_w = 15; // Ширина радиатора процессора
cpu_l = 15; // Длина радиатора процессора
cpu_h = h_t + base + pcb_thick; // Высота радиатора процессора
cpu_shift_w = 21.8; // Смещение радиатора процессора от правой стороны платы (сторона с двойной гребёнкой)
cpu_shift_l = 5.5; // Смещение радиатора процессора от задней стороны платы (сторона со слотом microSD)
cpu_gap = 1; // Зазор заглушки радиатора
cpu_d = 1; // Диаметр скругления заглушки радиатора
cpu_fix_w = 2; // Ширина крепёжной перемычки заглушки радиатора 

// Однорядная гребёнка
single_w = 4; // Ширина однорядной гребёнки
single_l = 35; // Длина однорядной гребёнки
single_h = h_t + base + pcb_thick; // Высота однорядной гребёнки
single_shift_l = 6.5; // Смещение однорядной гребёнки от задней стороны платы (сторона со слотом microSD)
single_d = 1; // Диаметр скругления углов однорядной гребёнки

// Двухрядная гребёнка
double_w = 6; // Ширина двухрядной гребёнки
double_l = 35; // Длина двухрядной гребёнки
double_h = h_t + base + pcb_thick; // Высота двухрядной гребёнки
double_shift_l = 6.5; // Смещение двухрядной гребёнки от задней стороны платы (сторона со слотом microSD)
double_gap = 1; // Зазор заглушки двухрядной гребёнки
double_d = 1; // Диаметр скругления заглушки двухрядной гребёнки
double_fix_w = 1.5; // Ширина крепёжной перемычки заглушки двухрядной гребёнки

// Разъём терминала
term_w = 3.5; // Ширина разъёма терминала
term_l = 8.5; // Длина разъёма терминала
term_h = h_t + base + pcb_thick; // Высота разъёма терминала
term_shift_w = 8; // Смещение разъёма терминала от правой стороны платы (сторона с двойной гребёнкой)

// Антенна
ann_d = 5 + 0.3; // Диаметр отверстия под антенну
ann_shift_w = 16; // Смещение центра отверстия под антенну от правой стороны платы (сторона с двойной гребёнкой)
ann_h = h_t + base + pcb_thick; // Высота антенны
ann_shift_l = 4.5; // Смещение центра отверстия под антенну от задней стороны платы (сторона со слотом microSD)

// Светодиоды
led_w = 8; // Ширина окна светодиод
led_h = 2; // Высота окна под светодиод
led_h1 = h_t + base + pcb_thick; // Вырез под светодиоды вверху
led_shift_w = 8; // Смещение окна светодиодов от правой стороны платы (сторона с двойной гребёнкой)


// Параметры решётки
gap_thick = 1; // Толщина внешнего обвода решётки
d_outer = 38; // Внешний диаметр решётки
d_inner = 34; // Внутренний диаметр решётки
d_center = 8; // Диаметр центральной части решётки
sectors =10; // Количество секторов решётки
d_sectors = 2; // Диаметр скругления углов решётки
fix_w = 3; // Ширина крепёжной перемычки
corner_full = 360/sectors; // Размер сектора одного окна с пермычкой
corner_win = corner_full/3; // Размер сектора одного окна

crn_base = 4; // Толшина нижней части кронштейна
crn_t = 5; // Толщина кронштейна
crn_gap = 5; // Зазор между кронштейном и корпусом
crn_h = h_b + h_t + 2*base; // Высота кронштейна
crn_pos = 1; // Положение кронштейна 0 - справа, 1 - слева

nut_outer_dia = 6.9; // Внешний диаметр гайки
nut_thickness = 2.4; // Глубина посадки гайки в поверхность




// Для печати комментируем/раскомментируем нужное 
difference() {
  top();
  translate([8, -9, 3]) {
    cylinder(r=1.6, h=10);
    translate([0, 10, 0]) {
      cylinder(r=1.6, h=10);
    }
  }
}
translate([-10, 8, wall])
        cube([wall, 18, h_b]);
//translate([10, 10, 6.6]) {
//  rotate([0, 0, 180]) {
//    lg_cam_attach();
//  }
//}
//translate([0,0, -h_b - base])
//bottom();
//translate([0,0, -h_b - base]) 
//bottom_fix();
//grill();
//pcb();

module bottom_fix () // Нижняя часть с кронштейном
{
difference()
{    
    union()
    {
        difference()
        {
            union() {
// Основной корпус
            hull()
            {
                for(x=[-(w/2 + delta_l + wall - r), (w/2 + delta_l + wall - r)])
                   for(y=[-(l/2 + delta_l + wall - r), (l/2 + delta_l + wall - r)])
                       translate([x,y,(base + h_b)/2]) cylinder(h=base + h_b, r=r, center=true);
            } // end hull
            
// Кронштейн
            mirror([crn_pos, 0, 0]) translate([-(w/2 + crn_t + crn_gap),-l/2,0]) cube([crn_t + crn_gap, l, crn_base]);
            mirror([crn_pos, 0, 0]) translate([-(w/2 + crn_t + crn_gap),-l/2,0]) cube([crn_t, l, crn_h]);            
        }
// Внутреннее пространство корпуса        
            hull()
            {
                for(x=[-(w/2 + delta_l - r_pcb), (w/2 + delta_l - r_pcb)])
                   for(y=[-(l/2 + delta_l - r_pcb), (l/2 + delta_l - r_pcb)])
                       translate([x,y,base + (h_b)/2]) cylinder(h=h_b, r=r_pcb, center=true);
            } // end hull
        } // end diff 
        cube([2, 12, h_b]);
        
// Стойки под плату        
        for(x=[-(w/2 - d_shift), (w/2 - d_shift)])
           for(y=[-(l/2 - d_shift), (l/2 - d_shift)])
               translate([x,y,base]) cylinder(h=thumb_b_h, r=d_shift+2*delta_l);
    } // end union
// Крепёжные отверстия кронштейна
    mirror([crn_pos, 0, 0]) for(y = [-l/4, 0, l/4]){
        translate([-(w/2 + crn_t + crn_gap), y,crn_base + nut_outer_dia/2]) rotate([0,90,0]) cylinder(d = d, h = crn_t);
        translate([-(w/2 + crn_gap + nut_thickness), y,crn_base + nut_outer_dia/2]) rotate([0,90,0]) cylinder(d = nut_outer_dia, h = nut_thickness, $fn = 6 );

    }       

    

// Отверстия под винты крепления
    for(x=[-(w/2 - d_shift), (w/2 - d_shift)])
       for(y=[-(l/2 - d_shift), (l/2 - d_shift)])
          translate([x,y,0]) cylinder(h=thumb_b_h+base, d=d);

// Отверстия под шляпки винтов
    for(x=[-(w/2 - d_shift), (w/2 - d_shift)])
       for(y=[-(l/2 - d_shift), (l/2 - d_shift)])
          translate([x,y,0]) cylinder(h=cap_h, d=cap_d);       
// Плата
    translate([0, 0 , base + thumb_b_h]) pcb();

// Решётка
    grill();

} // end diff    
    
} // end module bottom_fix




module top()
{
difference()
{    
    union()
    {
        difference()
        {
// Основной корпус
            hull()
            {
                for(x=[-(w/2 + delta_l + wall - r), (w/2 + delta_l + wall - r)])
                   for(y=[-(l/2 + delta_l + wall - r), (l/2 + delta_l + wall - r)])
                       translate([x,y,(base + h_t)/2]) cylinder(h=base + h_t, r=r, center=true);
            } // end hull
// Внутреннее пространство корпуса        
            hull()
            {
                for(x=[-(w/2 + delta_l - r_pcb), (w/2 + delta_l - r_pcb)])
                   for(y=[-(l/2 + delta_l - r_pcb), (l/2 + delta_l - r_pcb)])
                       translate([x,y,(h_t)/2]) cylinder(h=h_t, r=r_pcb, center=true);
            } // end hull
            

        } // end diff 
// Стойки под плату        
        for(x=[-(w/2 - d_shift), (w/2 - d_shift)])
           for(y=[-(l/2 - d_shift), (l/2 - d_shift)])
               translate([x,y,-h_b + thumb_b_h + pcb_thick + delta_l]) cylinder(h=h_t + h_b + base - thumb_b_h - pcb_thick - delta_l, r=d_shift+2*delta_l);
    } // end union    

// Отверстия под винты крепления
    for(x=[-(w/2 - d_shift), (w/2 - d_shift)])
       for(y=[-(l/2 - d_shift), (l/2 - d_shift)])
          translate([x,y,-h_b + thumb_b_h + pcb_thick]) cylinder(h=h_t + h_b - thumb_b_h - pcb_thick, d=d - delta_d);

// Выборка в стойках под плату
    hull()
    {
        for(x=[-(w/2 - d_shift - (d - delta_d)/2 ), (w/2 - d_shift - (d - delta_d)/2)])
           for(y=[-(l/2 - d_shift - (d - delta_d)/2), (l/2 - d_shift - (d - delta_d)/2)])
               translate([x,y,-h_b + thumb_b_h + pcb_thick + delta_l]) cylinder(h=1, r=d - delta_d);
    } // end hull
// Обвод по контуру для подрезания стоек
    difference()
    {
        hull()
        {
            for(x=[-(w/2 + delta_l + wall - r), (w/2 + delta_l + wall - r)])
               for(y=[-(l/2 + delta_l + wall - r), (l/2 + delta_l + wall - r)])
                   translate([x,y,-(h_b - thumb_b_h - pcb_thick - delta_l)/2]) cylinder(h=h_b - thumb_b_h - pcb_thick - delta_l, r=r, center=true);
        } // end hull
        hull()
        {
            for(x=[-(w/2 - r_pcb), (w/2 - r_pcb)])
               for(y=[-(l/2 - r_pcb), (l/2 - r_pcb)])
                   translate([x,y,-(h_b - thumb_b_h - pcb_thick - delta_l)/2]) cylinder(h=h_b - thumb_b_h - pcb_thick - delta_l, r=r_pcb, center=true);
        } // end hull
        
    } // end difference

// Текст
//translate([4.5, 7.5, h_t]) rotate([0,0,-90])
//linear_extrude(base)    
//   text("Z", font = "Cooper Black:style=Regular", halign = "left", valign = "bottom", size = 10 );
//translate([4.5, -3, h_t]) rotate([0,0,-90])
//linear_extrude(base)    
//   text("E", font = "Cooper Black:style=Regular", halign = "left", valign = "bottom", size = 6 );
//translate([4.5, -10, h_t]) rotate([0,0,-90])
//linear_extrude(base)    
//   text("r", font = "Cooper Black:style=Regular", halign = "left", valign = "bottom", size = 8 );
    
// Плата
   translate([0, 0, -(h_b - thumb_b_h)]) pcb();    

} // end diff    
    
} // end module top

module bottom ()
{
difference()
{    
    union()
    {
        difference()
        {
// Основной корпус
            hull()
            {
                for(x=[-(w/2 + delta_l + wall - r), (w/2 + delta_l + wall - r)])
                   for(y=[-(l/2 + delta_l + wall - r), (l/2 + delta_l + wall - r)])
                       translate([x,y,(base + h_b)/2]) cylinder(h=base + h_b, r=r, center=true);
            } // end hull
// Внутреннее пространство корпуса        
            hull()
            {
                for(x=[-(w/2 + delta_l - r_pcb), (w/2 + delta_l - r_pcb)])
                   for(y=[-(l/2 + delta_l - r_pcb), (l/2 + delta_l - r_pcb)])
                       translate([x,y,base + (h_b)/2]) cylinder(h=h_b, r=r_pcb, center=true);
            } // end hull
        } // end diff 
// Стойки под плату        
        for(x=[-(w/2 - d_shift), (w/2 - d_shift)])
           for(y=[-(l/2 - d_shift), (l/2 - d_shift)])
               translate([x,y,base]) cylinder(h=thumb_b_h, r=d_shift+2*delta_l);
    } // end union    

// Отверстия под винты крепления
    for(x=[-(w/2 - d_shift), (w/2 - d_shift)])
       for(y=[-(l/2 - d_shift), (l/2 - d_shift)])
          translate([x,y,0]) cylinder(h=thumb_b_h+base, d=d);

// Отверстия под шляпки винтов
    for(x=[-(w/2 - d_shift), (w/2 - d_shift)])
       for(y=[-(l/2 - d_shift), (l/2 - d_shift)])
          translate([x,y,0]) cylinder(h=cap_h, d=cap_d);       
// Плата
    translate([0, 0 , base + thumb_b_h]) pcb();

// Решётка
    grill();

} // end diff    
    
} // end module bottom



module grill()
{
difference()
{
    cylinder(d=d_outer + 2*gap_thick, h = base);
    cylinder(d=d_outer, h = base);
    for(z=[0,90,180,270])
        rotate([0,0,z])translate([-fix_w/2, -(d_outer/2 + 2*gap_thick), 0]) cube([fix_w, 3*gap_thick, base]);
}

    for (z=[0:corner_full:360]) rotate([0, 0, z])
    hull()
    {
        translate([0, (d_center + d_sectors)/2, 0]) cylinder(d=d_sectors, h = base);
        for (x=[-d_inner*tan(corner_win/2),d_inner*tan(corner_win/2)])
            translate([x, (d_inner - d_sectors)/2, 0]) cylinder(d=d_sectors, h = base);
    }  
} // end module grill

module pcb()
{
// Плата
hull()
{
    for(x=[-(w/2 + delta_l - r_pcb), (w/2 + delta_l - r_pcb)])
       for(y=[-(l/2 + delta_l - r_pcb), (l/2 + delta_l - r_pcb)])
           translate([x,y,pcb_thick/2]) cylinder(h=pcb_thick, r=r_pcb, center=true);
} // end hull

// Слот microSD
translate([(w/2 + delta_l - sd_shift_w - sd_w), -(l/2 + delta_l + wall), -sd_h ]) cube([sd_w,15 + wall + delta_l,sd_h]);

// Разъём microUSB
translate([(w/2 + delta_l - musb_shift_w - musb_w), -(l/2 + delta_l + wall), pcb_thick ]) cube([musb_w,7 + wall + delta_l,musb_h]);

// Разъём Ethernet
translate([(w/2 + delta_l - eth_shift_w - eth_w), (l/2 - 15 + delta_l + wall), pcb_thick ]) cube([eth_w, 15, eth_h]);

// Разъём USB
translate([(w/2 + delta_l - usb_shift_w - usb_w), (l/2 - 13 + delta_l + wall), pcb_thick + 0.5]) cube([usb_w, 13, usb_h]);

// Радиатор процессора
difference(){
  // removed top above radiator
//    translate([(w/2 - cpu_shift_w - cpu_w), -l/2 + cpu_shift_l, pcb_thick ]) cube([cpu_w, cpu_l, cpu_h]);
    hull()
    {
    for(x=[(w/2 - cpu_shift_w - cpu_w + cpu_d/2 + cpu_gap), (w/2 - cpu_shift_w - cpu_d/2 - cpu_gap)])
        for(y=[(-l/2 + cpu_shift_l + cpu_l - cpu_d/2 - cpu_gap), (-l/2 + cpu_shift_l + cpu_d/2 + cpu_gap)])
            translate([x, y, pcb_thick]) cylinder(d=cpu_d, h=cpu_h);
    } // end hull
for(x=[(w/2 - cpu_shift_w - cpu_w), (w/2 - cpu_shift_w - cpu_gap)])
    translate([x, (-l/2 + cpu_shift_l + cpu_l/2 - cpu_fix_w/2) , pcb_thick]) cube([cpu_gap, cpu_fix_w, cpu_h]);

for(y=[(-l/2 + cpu_shift_l), (-l/2 + cpu_shift_l + cpu_l - cpu_gap)])
        translate([(w/2 - cpu_shift_w - cpu_w/2 - cpu_fix_w/2), y, pcb_thick]) cube([cpu_fix_w, cpu_gap, cpu_h]);
} // end diff

// Однорядная гребёнка
hull()
{
for(x=[-(w/2 + delta_l - single_d/2), -(w/2 + delta_l - single_w + single_d/2)])
    for(y=[(-l/2 + single_shift_l + single_d/2), (-l/2 + single_shift_l + single_l - single_d/2)])
        translate([x, y, pcb_thick]) cylinder(d=single_d, h=single_h);
} // end hull

// Двухрядная гребёнка 
difference(){
    translate([(w/2 + delta_l - double_w), -l/2 + double_shift_l, pcb_thick ]) cube([double_w, double_l, double_h]);
    hull()
    {
    for(x=[(w/2 + delta_l - double_w + double_d/2 + double_gap), (w/2 + delta_l - double_d/2 - double_gap)])
        for(y=[(-l/2 + double_shift_l + double_l - double_d/2 - double_gap), (-l/2 + double_shift_l + double_d/2 + double_gap)])
            translate([x, y, pcb_thick]) cylinder(d=double_d, h=double_h);
    } // end hull

for(y=[(-l/2 + double_shift_l), (-l/2 + double_shift_l + double_l - double_gap)])
        translate([(w/2 + delta_l - double_w/2 - double_fix_w/2), y, pcb_thick]) cube([double_fix_w, double_gap, double_h]);
} // end diff

// Разъём терминала
translate([(w/2 - term_shift_w - term_w), l/2 + delta_l- term_l, pcb_thick ]) cube([term_w, term_l , term_h]);
// Антенна
//translate([(w/2 - ann_shift_w), -l/2 + ann_shift_l, pcb_thick ]) cylinder(d = ann_d, h = ann_h);

// Светодиоды
translate([(w/2 + delta_l - led_shift_w - led_w), -(l/2 + delta_l + wall), pcb_thick ]) cube([led_w,2 + wall + delta_l,led_h]);
translate([(w/2 + delta_l - led_shift_w - led_w), -(l/2 + delta_l), pcb_thick ]) cube([led_w, 2,led_h1]);


    
} // end module pcb





