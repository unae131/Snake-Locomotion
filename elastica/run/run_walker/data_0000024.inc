#declare Ribbon_Spline0 =
spline { natural_spline
-0.25, <0.000142983,-0.117032,2.77071>,
0, <0.000268821,-0.980557,2.91196>,
0.25, <0.000394693,-1.84409,3.0532>,
0.5, <0.000520608,-2.70711,3.19424>,
0.75, <0.000645907,-3.57088,3.33069>,
1, <0.000769853,-4.43518,3.4593>,
1.25, <0.000889213,-5.30275,3.56367>,
1.5, <0.00100368,-6.17294,3.6433>,
1.75, <0.00111314,-7.04507,3.69775>,
2, <0.00121715,-7.91843,3.72664>,
2.25, <0.00131525,-8.79225,3.72964>,
2.5, <0.00140749,-9.66577,3.7065>,
2.75, <0.00149386,-10.5382,3.65704>,
3, <0.00157832,-11.4107,3.59887>,
3.25, <0.00166157,-12.2829,3.53547>,
3.5, <0.00174373,-13.1552,3.46663>,
3.75, <0.00182581,-14.0275,3.39778>
}
union{
#local Nr = -0.25;
#local EndNr = 3.7;
#while (Nr <= EndNr)
box{ <-1.75,0,0>,
<5.25,-0.3,0.68>
pigment{ color rgbt<0.45,0.39,1,0>}
rotate x*0
translate Ribbon_Spline0(Nr)
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
#local Nr = Nr + 0.04;
#end
}
#declare Ribbon_Spline2 =
spline { natural_spline
-0.25, <0.000394694,-1.84409,3.0532>,
0, <0.000374415,-1.92217,2.59312>,
0.25, <0.000354138,-2.00032,2.13307>,
0.5, <0.000333866,-2.07851,1.67302>,
0.75, <0.000313582,-2.15668,1.21299>,
1, <0.000293294,-2.23482,0.75297>,
1.25, <0.000273002,-2.31296,0.292958>
}
union{
#local Nr = -0.25;
#local EndNr = 1.2;
#while (Nr <= EndNr)
box{ <-1.75,1.75,0>,
<5.25,-1.75,0.3>
pigment{ color rgbt<0.45,0.39,1,0>}
rotate -x*9.64069
translate Ribbon_Spline2(Nr)
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
#local Nr = Nr + 0.04;
#end
}
#declare Ribbon_Spline3 =
spline { natural_spline
-0.25, <0.00166157,-12.2829,3.53547>,
0, <0.00161063,-12.2351,2.97085>,
0.25, <0.00155967,-12.1871,2.40625>,
0.5, <0.00150871,-12.1391,1.84166>,
0.75, <0.00145777,-12.0911,1.27709>,
1, <0.00140687,-12.0432,0.71253>,
1.25, <0.00135598,-11.9952,0.147986>
}
union{
#local Nr = -0.25;
#local EndNr = 1.2;
#while (Nr <= EndNr)
box{ <-1.75,1.75,0>,
<5.25,-1.75,0.3>
pigment{ color rgbt<0.45,0.39,1,0>}
rotate x*4.85498
translate Ribbon_Spline3(Nr)
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
#local Nr = Nr + 0.04;
#end
}
sphere_sweep
{
b_spline
25,
<-1.47314,-14.1503,1.5>, 0.313003,
<-0.541145,-14.1503,1.5>, 0.313004,
<0.390842,-14.1503,1.5>, 0.313005,
<1.32282,-14.1503,1.5>, 0.313006,
<2.2548,-14.1503,1.5>, 0.313005,
<3.18679,-14.1503,1.5>, 0.313004,
<4.11878,-14.1503,1.5>, 0.313006,
<5.05074,-14.1503,1.5>, 0.310357,
<5.50967,-13.2378,1.5>, 0.309197,
<5.54696,-12.2365,1.5>, 0.31066,
<5.5167,-11.2347,1.5>, 0.310637,
<5.51769,-10.2324,1.5>, 0.297998,
<4.45005,-9.88949,1.5>, 0.285859,
<3.39008,-9.54856,1.5>, 0.287269,
<2.29743,-9.42571,1.5>, 0.288278,
<1.19951,-9.42765,1.5>, 0.288268,
<0.107395,-9.55641,1.5>, 0.287187,
<-0.949842,-9.90928,1.5>, 0.285731,
<-2.01499,-10.2627,1.5>, 0.297754,
<-2.0189,-11.2674,1.5>, 0.310259,
<-2.05286,-12.2716,1.5>, 0.310292,
<-2.00623,-13.2747,1.5>, 0.308748,
<-1.47314,-14.1503,1.5>, 0.298579,
<-0.541751,-13.5923,1.5>, 0.289997,
<0.388291,-13.0321,1.5>, 0.289999
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
25,
<-1.46307,-14.0959,0.9>, 0.313375,
<-0.533286,-14.0959,0.9>, 0.313376,
<0.39649,-14.096,0.9>, 0.313377,
<1.32626,-14.096,0.9>, 0.313378,
<2.25603,-14.0961,0.9>, 0.313377,
<3.18581,-14.0961,0.9>, 0.313376,
<4.11559,-14.0961,0.9>, 0.313378,
<5.04534,-14.0962,0.9>, 0.311119,
<5.50514,-13.1926,0.9>, 0.310337,
<5.54467,-12.1987,0.9>, 0.311784,
<5.51118,-11.2041,0.9>, 0.311758,
<5.50838,-10.2091,0.9>, 0.299577,
<4.45005,-9.88949,0.9>, 0.286877,
<3.39007,-9.54856,0.9>, 0.287269,
<2.29743,-9.42571,0.9>, 0.288278,
<1.19951,-9.42765,0.9>, 0.288268,
<0.107395,-9.55642,0.9>, 0.287187,
<-0.949842,-9.90928,0.9>, 0.286719,
<-2.0058,-10.2411,0.9>, 0.29927,
<-2.0141,-11.2389,0.9>, 0.311317,
<-2.05179,-12.2361,0.9>, 0.311356,
<-2.00186,-13.2323,0.9>, 0.309821,
<-1.46307,-14.0959,0.9>, 0.299118,
<-0.537546,-13.5282,0.9>, 0.289997,
<0.386602,-12.9584,0.9>, 0.289999
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
25,
<-1.4884,-0.0461827,1.5>, 0.312412,
<-0.552875,-0.0459802,1.5>, 0.312413,
<0.382641,-0.0457776,1.5>, 0.312414,
<1.31815,-0.0455807,1.5>, 0.312415,
<2.25366,-0.0453814,1.5>, 0.312414,
<3.18918,-0.0451821,1.5>, 0.312413,
<4.1247,-0.0449829,1.5>, 0.312416,
<5.06019,-0.044785,1.5>, 0.30963,
<5.53438,-0.956011,1.5>, 0.308483,
<5.56808,-1.96103,1.5>, 0.310094,
<5.53371,-2.96643,1.5>, 0.310071,
<5.53027,-3.97232,1.5>, 0.297202,
<4.45618,-4.32166,1.5>, 0.285192,
<3.39311,-4.66079,1.5>, 0.28696,
<2.29799,-4.78285,1.5>, 0.287968,
<1.19771,-4.78101,1.5>, 0.287957,
<0.103076,-4.65324,1.5>, 0.286875,
<-0.957402,-4.30241,1.5>, 0.285067,
<-2.02919,-3.94352,1.5>, 0.296969,
<-2.03768,-2.93529,1.5>, 0.309705,
<-2.07602,-1.92765,1.5>, 0.309739,
<-2.03401,-0.920707,1.5>, 0.308047,
<-1.4884,-0.0461827,1.5>, 0.298155,
<-0.562902,-0.613885,1.5>, 0.289997,
<0.361289,-1.18368,1.5>, 0.289999
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
25,
<-1.4663,-0.141191,0.9>, 0.31326,
<-0.535831,-0.140962,0.9>, 0.313261,
<0.394628,-0.140736,0.9>, 0.313262,
<1.32508,-0.140509,0.9>, 0.313263,
<2.25553,-0.140283,0.9>, 0.313262,
<3.18599,-0.140057,0.9>, 0.313261,
<4.11646,-0.139832,0.9>, 0.313264,
<5.04689,-0.139607,0.9>, 0.311167,
<5.51718,-1.03624,0.9>, 0.310622,
<5.5577,-2.02782,0.9>, 0.312141,
<5.52125,-3.02002,0.9>, 0.312112,
<5.51452,-4.01278,0.9>, 0.299952,
<4.45618,-4.32166,0.9>, 0.286921,
<3.39311,-4.66079,0.9>, 0.28696,
<2.29799,-4.78285,0.9>, 0.287968,
<1.19771,-4.78101,0.9>, 0.287957,
<0.103076,-4.65324,0.9>, 0.286875,
<-0.957402,-4.30241,0.9>, 0.286753,
<-2.01346,-3.98079,0.9>, 0.299621,
<-2.02632,-2.98507,0.9>, 0.311638,
<-2.06759,-1.99003,0.9>, 0.311681,
<-2.01681,-0.996074,0.9>, 0.31007,
<-1.4663,-0.141191,0.9>, 0.299203,
<-0.548691,-0.721564,0.9>, 0.289997,
<0.367547,-1.30406,0.9>, 0.289999
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
13,
<3.39311,-4.66079,1.5>, 0.240359,
<3.39288,-5.06811,1.5>, 0.240359,
<3.39264,-5.47542,1.5>, 0.240359,
<3.39241,-5.88274,1.5>, 0.240359,
<3.39216,-6.29005,1.5>, 0.24036,
<3.39191,-6.69736,1.5>, 0.24036,
<3.39167,-7.10467,1.5>, 0.24036,
<3.39141,-7.51199,1.5>, 0.24036,
<3.39115,-7.9193,1.5>, 0.240359,
<3.39089,-8.32661,1.5>, 0.240359,
<3.39062,-8.73392,1.5>, 0.240359,
<3.39035,-9.14124,1.5>, 0.24036,
<3.39008,-9.54855,1.5>, 0.240359
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
13,
<2.29799,-4.78285,1.5>, 0.246616,
<2.29796,-5.16976,1.5>, 0.246617,
<2.29793,-5.55667,1.5>, 0.246617,
<2.29789,-5.94357,1.5>, 0.246617,
<2.29785,-6.33048,1.5>, 0.246618,
<2.29781,-6.71738,1.5>, 0.246618,
<2.29777,-7.10428,1.5>, 0.246618,
<2.29772,-7.49119,1.5>, 0.246617,
<2.29766,-7.87809,1.5>, 0.246617,
<2.29761,-8.26499,1.5>, 0.246617,
<2.29755,-8.6519,1.5>, 0.246617,
<2.29749,-9.0388,1.5>, 0.246617,
<2.29743,-9.42571,1.5>, 0.246617
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
13,
<1.19771,-4.78101,1.5>, 0.246516,
<1.19785,-5.16823,1.5>, 0.246517,
<1.19798,-5.55546,1.5>, 0.246517,
<1.19812,-5.94268,1.5>, 0.246517,
<1.19827,-6.3299,1.5>, 0.246518,
<1.19841,-6.71711,1.5>, 0.246518,
<1.19856,-7.10433,1.5>, 0.246518,
<1.19871,-7.49155,1.5>, 0.246517,
<1.19887,-7.87877,1.5>, 0.246517,
<1.19902,-8.26598,1.5>, 0.246517,
<1.19919,-8.65321,1.5>, 0.246517,
<1.19935,-9.04043,1.5>, 0.246517,
<1.19951,-9.42765,1.5>, 0.246516
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
13,
<0.103076,-4.65325,1.5>, 0.239981,
<0.103419,-5.06184,1.5>, 0.239981,
<0.103761,-5.47044,1.5>, 0.239982,
<0.104104,-5.87904,1.5>, 0.239981,
<0.104458,-6.28764,1.5>, 0.239982,
<0.104812,-6.69623,1.5>, 0.239982,
<0.105166,-7.10483,1.5>, 0.239982,
<0.105532,-7.51343,1.5>, 0.239982,
<0.105898,-7.92202,1.5>, 0.239981,
<0.106264,-8.33062,1.5>, 0.239982,
<0.106641,-8.73922,1.5>, 0.239981,
<0.107018,-9.14781,1.5>, 0.239982,
<0.107395,-9.55641,1.5>, 0.239981
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}