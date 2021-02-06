package frc.paths;

import com.team319.trajectory.Path;

public class path1 extends Path {
   // dt,x,y,left.pos,left.vel,left.acc,left.jerk,center.pos,center.vel,center.acc,center.jerk,right.pos,right.vel,right.acc,right.jerk,heading
	private static final double[][] points = {
				{0.0200,41.0772,4.8600,0.0028,0.1400,7.0000,0.0000,0.0028,0.1400,7.0000,0.0000,0.0028,0.1400,7.0000,0.0000,-3.1416},
				{0.0200,41.0716,4.8600,0.0084,0.2801,7.0054,0.2687,0.0084,0.2800,7.0000,0.0000,0.0084,0.2799,6.9946,-0.2687,-3.1416},
				{0.0200,41.0632,4.8600,0.0168,0.4204,7.0127,0.3670,0.0168,0.4200,7.0000,0.0000,0.0168,0.4196,6.9873,-0.3670,-3.1416},
				{0.0200,41.0520,4.8600,0.0280,0.5609,7.0246,0.5956,0.0280,0.5600,7.0000,0.0000,0.0280,0.5591,6.9754,-0.5956,-3.1416},
				{0.0200,41.0380,4.8600,0.0421,0.7017,7.0403,0.7841,0.0420,0.7000,7.0000,0.0000,0.0419,0.6983,6.9597,-0.7841,-3.1416},
				{0.0200,41.0212,4.8600,0.0589,0.8429,7.0596,0.9640,0.0588,0.8400,7.0000,0.0000,0.0587,0.8371,6.9404,-0.9640,-3.1416},
				{0.0200,41.0016,4.8600,0.0786,0.9845,7.0823,1.1331,0.0784,0.9800,7.0000,0.0000,0.0782,0.9755,6.9177,-1.1331,-3.1417},
				{0.0200,40.9792,4.8600,0.1011,1.1267,7.1080,1.2892,0.1008,1.1200,7.0000,0.0000,0.1005,1.1133,6.8920,-1.2892,-3.1417},
				{0.0200,40.9540,4.8600,0.1265,1.2694,7.1366,1.4301,0.1260,1.2600,7.0000,0.0000,0.1255,1.2506,6.8634,-1.4301,-3.1418},
				{0.0200,40.9260,4.8600,0.1548,1.4127,7.1677,1.5537,0.1540,1.4000,7.0000,0.0000,0.1532,1.3873,6.8323,-1.5537,-3.1419},
				{0.0200,40.8952,4.8600,0.1859,1.5568,7.2009,1.6579,0.1848,1.5400,7.0000,0.0000,0.1837,1.5232,6.7991,-1.6579,-3.1420},
				{0.0200,40.8616,4.8600,0.2199,1.7015,7.2357,1.7405,0.2184,1.6800,7.0000,0.0000,0.2169,1.6585,6.7643,-1.7405,-3.1422},
				{0.0200,40.8252,4.8601,0.2569,1.8469,7.2717,1.7995,0.2548,1.8200,7.0000,0.0000,0.2527,1.7931,6.7283,-1.7995,-3.1424},
				{0.0200,40.7860,4.8601,0.2967,1.9931,7.3083,1.8331,0.2940,1.9600,7.0000,0.0000,0.2913,1.9269,6.6917,-1.8331,-3.1427},
				{0.0200,40.7440,4.8602,0.3395,2.1400,7.3451,1.8393,0.3360,2.1000,7.0000,0.0000,0.3325,2.0600,6.6549,-1.8393,-3.1430},
				{0.0200,40.6992,4.8602,0.3853,2.2876,7.3814,1.8166,0.3808,2.2400,7.0000,0.0000,0.3763,2.1924,6.6186,-1.8166,-3.1434},
				{0.0200,40.6516,4.8603,0.4340,2.4359,7.4167,1.7634,0.4284,2.3800,7.0000,0.0000,0.4228,2.3241,6.5833,-1.7634,-3.1438},
				{0.0200,40.6012,4.8605,0.4857,2.5849,7.4503,1.6785,0.4788,2.5200,7.0000,0.0000,0.4719,2.4551,6.5497,-1.6785,-3.1444},
				{0.0200,40.5480,4.8606,0.5404,2.7346,7.4815,1.5608,0.5320,2.6600,7.0000,0.0000,0.5236,2.5854,6.5185,-1.5608,-3.1450},
				{0.0200,40.4920,4.8608,0.5981,2.8848,7.5097,1.4097,0.5880,2.8000,7.0000,0.0000,0.5779,2.7152,6.4903,-1.4097,-3.1456},
				{0.0200,40.4332,4.8611,0.6588,3.0355,7.5342,1.2248,0.6468,2.9400,7.0000,0.0000,0.6348,2.8445,6.4658,-1.2248,-3.1464},
				{0.0200,40.3716,4.8614,0.7225,3.1865,7.5543,1.0059,0.7084,3.0800,7.0000,0.0000,0.6943,2.9735,6.4457,-1.0059,-3.1473},
				{0.0200,40.3072,4.8618,0.7893,3.3379,7.5694,0.7534,0.7728,3.2200,7.0000,0.0000,0.7563,3.1021,6.4306,-0.7534,-3.1482},
				{0.0200,40.2400,4.8623,0.8591,3.4895,7.5787,0.4678,0.8400,3.3600,7.0000,0.0000,0.8209,3.2305,6.4213,-0.4678,-3.1492},
				{0.0200,40.1700,4.8629,0.9319,3.6411,7.5817,0.1503,0.9100,3.5000,7.0000,0.0000,0.8881,3.3589,6.4183,-0.1503,-3.1504},
				{0.0200,40.0972,4.8635,1.0078,3.7927,7.5778,-0.1979,0.9828,3.6400,7.0000,0.0000,0.9578,3.4873,6.4222,0.1979,-3.1516},
				{0.0200,40.0216,4.8643,1.0866,3.9440,7.5663,-0.5750,1.0584,3.7800,7.0000,0.0000,1.0302,3.6160,6.4337,0.5750,-3.1529},
				{0.0200,39.9432,4.8653,1.1685,4.0950,7.5467,-0.9791,1.1368,3.9200,7.0000,0.0000,1.1051,3.7450,6.4533,0.9790,-3.1543},
				{0.0200,39.8620,4.8664,1.2535,4.2453,7.5185,-1.4078,1.2180,4.0600,7.0000,0.0000,1.1825,3.8747,6.4815,1.4078,-3.1558},
				{0.0200,39.7780,4.8676,1.3414,4.3950,7.4814,-1.8587,1.3020,4.2000,7.0000,0.0000,1.2626,4.0050,6.5186,1.8587,-3.1573},
				{0.0200,39.6912,4.8691,1.4322,4.5436,7.4348,-2.3292,1.3888,4.3400,7.0000,0.0000,1.3454,4.1364,6.5652,2.3292,-3.1590},
				{0.0200,39.6017,4.8707,1.5261,4.6912,7.3784,-2.8168,1.4784,4.4800,7.0000,0.0000,1.4307,4.2688,6.6215,2.8169,-3.1607},
				{0.0200,39.5093,4.8725,1.6228,4.8375,7.3121,-3.3189,1.5708,4.6200,7.0000,0.0000,1.5188,4.4025,6.6879,3.3190,-3.1624},
				{0.0200,39.4141,4.8746,1.7224,4.9822,7.2354,-3.8331,1.6660,4.7600,7.0000,0.0000,1.6096,4.5378,6.7646,3.8332,-3.1642},
				{0.0200,39.3161,4.8769,1.8249,5.1251,7.1483,-4.3574,1.7640,4.9000,7.0000,0.0000,1.7031,4.6749,6.8517,4.3574,-3.1660},
				{0.0200,39.2154,4.8795,1.9303,5.2661,7.0505,-4.8899,1.8648,5.0400,7.0000,0.0000,1.7993,4.8139,6.9495,4.8900,-3.1678},
				{0.0200,39.1118,4.8823,2.0384,5.4050,6.9419,-5.4296,1.9684,5.1800,7.0000,0.0000,1.8984,4.9550,7.0581,5.4297,-3.1696},
				{0.0200,39.0054,4.8853,2.1492,5.5414,6.8223,-5.9759,2.0748,5.3200,7.0000,0.0000,2.0004,5.0986,7.1777,5.9760,-3.1714},
				{0.0200,38.8963,4.8887,2.2627,5.6753,6.6918,-6.5291,2.1840,5.4600,7.0000,0.0000,2.1053,5.2447,7.3082,6.5292,-3.1731},
				{0.0200,38.7844,4.8923,2.3788,5.8063,6.5500,-7.0904,2.2960,5.6000,7.0000,0.0000,2.2132,5.3937,7.4500,7.0904,-3.1747},
				{0.0200,38.6696,4.8962,2.4975,5.9342,6.3967,-7.6618,2.4108,5.7400,7.0000,0.0000,2.3241,5.5458,7.6033,7.6619,-3.1763},
				{0.0200,38.5521,4.9004,2.6187,6.0588,6.2318,-8.2468,2.5284,5.8800,7.0000,0.0000,2.4381,5.7012,7.7682,8.2469,-3.1777},
				{0.0200,38.4318,4.9048,2.7423,6.1799,6.0548,-8.8500,2.6488,6.0200,7.0000,0.0000,2.5553,5.8601,7.9452,8.8501,-3.1790},
				{0.0200,38.3087,4.9095,2.8682,6.2972,5.8652,-9.4776,2.7720,6.1600,7.0000,0.0000,2.6758,6.0228,8.1348,9.4777,-3.1801},
				{0.0200,38.1828,4.9144,2.9964,6.4105,5.6625,-10.1373,2.8980,6.3000,7.0000,0.0000,2.7996,6.1895,8.3375,10.1373,-3.1810},
				{0.0200,38.0541,4.9195,3.1268,6.5194,5.4457,-10.8384,3.0268,6.4400,7.0000,0.0000,2.9268,6.3606,8.5543,10.8384,-3.1816},
				{0.0200,37.9226,4.9248,3.2593,6.6237,5.2139,-11.5925,3.1584,6.5800,7.0000,0.0000,3.0575,6.5363,8.7861,11.5924,-3.1820},
				{0.0200,37.7883,4.9302,3.3938,6.7230,4.9656,-12.4132,3.2928,6.7200,7.0000,0.0000,3.1918,6.7170,9.0344,12.4130,-3.1820},
				{0.0200,37.6512,4.9357,3.5301,6.8170,4.6993,-13.3166,3.4300,6.8600,7.0000,0.0000,3.3299,6.9030,9.3007,13.3163,-3.1816},
				{0.0200,37.5113,4.9413,3.6682,6.9052,4.4128,-14.3216,3.5700,7.0000,7.0000,0.0000,3.4718,7.0948,9.5871,14.3211,-3.1809},
				{0.0200,37.3686,4.9468,3.8080,6.9873,4.1038,-15.4503,3.7128,7.1400,7.0000,0.0000,3.6176,7.2927,9.8961,15.4496,-3.1797},
				{0.0200,37.2231,4.9522,3.9492,7.0627,3.7693,-16.7286,3.8584,7.2800,7.0000,0.0000,3.7676,7.4973,10.2307,16.7276,-3.1779},
				{0.0200,37.0748,4.9574,4.0918,7.1308,3.4055,-18.1865,4.0068,7.4200,7.0000,0.0000,3.9218,7.7092,10.5944,18.1852,-3.1756},
				{0.0200,36.9237,4.9624,4.2356,7.1910,3.0084,-19.8590,4.1580,7.5600,7.0000,0.0000,4.0804,7.9290,10.9915,19.8572,-3.1726},
				{0.0200,36.7697,4.9669,4.3805,7.2424,2.5726,-21.7868,4.3120,7.7000,7.0000,0.0000,4.2435,8.1576,11.4272,21.7845,-3.1690},
				{0.0200,36.6130,4.9708,4.5262,7.2843,2.0923,-24.0174,4.4688,7.8400,7.0000,0.0000,4.4114,8.3957,11.9075,24.0144,-3.1645},
				{0.0200,36.4534,4.9741,4.6725,7.3155,1.5602,-26.6055,4.6284,7.9800,7.0000,0.0000,4.5843,8.6445,12.4395,26.6016,-3.1592},
				{0.0200,36.2910,4.9764,4.8192,7.3348,0.9679,-29.6146,4.7908,8.1200,7.0000,0.0000,4.7624,8.9052,13.0317,29.6095,-3.1529},
				{0.0200,36.1259,4.9777,4.9660,7.3409,0.3055,-33.1172,4.9560,8.2600,7.0000,0.0000,4.9460,9.1790,13.6940,33.1107,-3.1456},
				{0.0200,35.9579,4.9777,5.1126,7.3322,-0.4384,-37.1957,5.1240,8.4000,7.0000,0.0000,5.1354,9.4678,14.4377,37.1873,-3.1371},
				{0.0200,35.7871,4.9761,5.2588,7.3066,-1.2772,-41.9416,5.2948,8.5400,7.0000,0.0000,5.3308,9.7733,15.2763,41.9307,-3.1272},
				{0.0200,35.6135,4.9726,5.4040,7.2621,-2.2263,-47.4533,5.4684,8.6800,7.0000,0.0000,5.5328,10.0978,16.2251,47.4393,-3.1158},
				{0.0200,35.4372,4.9670,5.5479,7.1960,-3.3029,-53.8317,5.6448,8.8200,7.0000,0.0000,5.7417,10.4438,17.3014,53.8138,-3.1028},
				{0.0200,35.2582,4.9587,5.6900,7.1055,-4.5263,-61.1704,5.8240,8.9600,7.0000,0.0000,5.9579,10.8143,18.5243,61.1470,-3.0880},
				{0.0200,35.0765,4.9475,5.8298,6.9872,-5.9171,-69.5373,6.0060,9.1000,7.0000,0.0000,6.1822,11.2126,19.9145,69.5073,-3.0711},
				{0.0200,34.8923,4.9327,5.9665,6.8372,-7.4960,-78.9455,6.1908,9.2400,7.0000,0.0000,6.4150,11.6425,21.4926,78.9066,-3.0519},
				{0.0200,34.7057,4.9139,6.0996,6.6516,-9.2820,-89.3019,6.3784,9.3800,7.0000,0.0000,6.6572,12.1080,23.2776,89.2518,-3.0301},
				{0.0200,34.5167,4.8904,6.2281,6.4258,-11.2886,-100.3272,6.5688,9.5200,7.0000,0.0000,6.9095,12.6137,25.2829,100.2628,-3.0053},
				{0.0200,34.3257,4.8616,6.3512,6.1555,-13.5172,-111.4333,6.7620,9.6600,7.0000,0.0000,7.1728,13.1639,27.5099,111.3510,-2.9773},
				{0.0200,34.1329,4.8265,6.4679,5.8365,-15.9481,-121.5441,6.9580,9.8000,7.0000,0.0000,7.4480,13.7626,29.9387,121.4400,-2.9456},
				{0.0200,33.9386,4.7843,6.5772,5.4660,-18.5252,-128.8560,7.1568,9.9400,7.0000,0.0000,7.7363,14.4129,32.5132,128.7268,-2.9098},
				{0.0200,33.7434,4.7342,6.6781,5.0433,-21.1363,-130.5553,7.3584,10.0800,7.0000,0.0000,8.0386,15.1153,35.1212,130.3990,-2.8695},
				{0.0200,33.5478,4.6749,6.7695,4.5715,-23.5878,-122.5713,7.5628,10.2200,7.0000,0.0000,8.3559,15.8667,37.5690,122.3898,-2.8243},
				{0.0200,33.3526,4.6054,6.8507,4.0600,-25.5788,-99.5533,7.7700,10.3600,7.0000,0.0000,8.6891,16.6578,39.5561,99.3565,-2.7739},
				{0.0200,33.1588,4.5245,6.9213,3.5262,-26.6872,-55.4204,7.9800,10.5000,7.0000,0.0000,9.0385,17.4711,40.6607,55.2290,-2.7181},
				{0.0200,32.9725,4.4339,6.9798,2.9266,-29.9835,-164.8123,8.1872,10.3600,-7.0000,0.0000,9.3943,17.7904,15.9670,-1234.6864,-2.6587},
				{0.0200,32.7946,4.3334,7.0280,2.4086,-25.8967,204.3411,8.3916,10.2200,-7.0000,0.0000,9.7549,18.0281,11.8830,-204.1997,-2.5962},
				{0.0200,32.6257,4.2234,7.0680,2.0010,-20.3798,275.8419,8.5932,10.0800,-7.0000,0.0000,10.1180,18.1555,6.3706,-275.6181,-2.5315},
				{0.0200,32.4666,4.1042,7.1025,1.7259,-13.7556,331.2112,8.7920,9.9400,-7.0000,0.0000,10.4810,18.1505,-0.2478,-330.9234,-2.4658},
				{0.0200,32.3178,3.9767,7.1344,1.5942,-6.5835,358.6053,8.9880,9.8000,-7.0000,0.0000,10.8410,18.0022,-7.4136,-358.2905,-2.4002},
				{0.0200,32.1796,3.8417,7.1665,1.6032,0.4483,351.5924,9.1812,9.6600,-7.0000,0.0000,11.1953,17.7134,-14.4396,-351.2971,-2.3357},
				{0.0200,32.0521,3.7003,7.2012,1.7369,6.6843,311.8000,9.3716,9.5200,-7.0000,0.0000,11.5413,17.3000,-20.6709,-311.5654,-2.2735},
				{0.0200,31.9352,3.5537,7.2406,1.9699,11.6489,248.2267,9.5592,9.3800,-7.0000,0.0000,11.8770,16.7874,-25.6324,-248.0764,-2.2142},
				{0.0200,31.8286,3.4028,7.2860,2.2723,15.1220,173.6544,9.7440,9.2400,-7.0000,0.0000,12.2011,16.2053,-29.1042,-173.5903,-2.1585},
				{0.0200,31.7317,3.2487,7.3383,2.6149,17.1279,100.2989,9.9260,9.1000,-7.0000,0.0000,12.5128,15.5831,-31.1104,-100.3075,-2.1066},
				{0.0200,31.6440,3.0925,7.3978,2.9722,17.8642,36.8104,10.1052,8.9600,-7.0000,0.0000,12.8117,14.9461,-31.8478,-36.8694,-2.0587},
				{0.0200,31.5648,2.9349,7.4643,3.3244,17.6123,-12.5944,10.2816,8.8200,-7.0000,0.0000,13.0980,14.3142,-31.5976,12.5063,-2.0147},
				{0.0200,31.4934,2.7766,7.5374,3.6577,16.6647,-47.3758,10.4552,8.6800,-7.0000,0.0000,13.3720,13.7011,-30.6521,47.2766,-1.9745},
				{0.0200,31.4293,2.6183,7.6167,3.9633,15.2794,-69.2679,10.6260,8.5400,-7.0000,0.0000,13.6343,13.1158,-29.2687,69.1694,-1.9379},
				{0.0200,31.3716,2.4606,7.7014,4.2365,13.6599,-80.9743,10.7940,8.4000,-7.0000,0.0000,13.8856,12.5627,-27.6510,80.8840,-1.9046},
				{0.0200,31.3199,2.3037,7.7909,4.4756,11.9542,-85.2838,10.9592,8.2600,-7.0000,0.0000,14.1265,12.0438,-25.9469,85.2043,-1.8743},
				{0.0200,31.2735,2.1480,7.8845,4.6808,10.2620,-84.6103,11.1216,8.1200,-7.0000,0.0000,14.3577,11.5587,-24.2561,84.5429,-1.8468},
				{0.0200,31.2320,1.9939,7.9816,4.8537,8.6453,-80.8373,11.2812,7.9800,-7.0000,0.0000,14.5798,11.1059,-22.6405,80.7812,-1.8218},
				{0.0200,31.1948,1.8416,8.0815,4.9965,7.1387,-75.3305,11.4380,7.8400,-7.0000,0.0000,14.7934,10.6832,-21.1348,75.2846,-1.7991},
				{0.0200,31.1615,1.6913,8.1838,5.1116,5.7582,-69.0245,11.5920,7.7000,-7.0000,0.0000,14.9992,10.2881,-19.7550,68.9872,-1.7784},
				{0.0200,31.1318,1.5430,8.2878,5.2018,4.5077,-62.5253,11.7432,7.5600,-7.0000,0.0000,15.1976,9.9180,-18.5051,62.4951,-1.7595},
				{0.0200,31.1052,1.3970,8.3932,5.2695,3.3836,-56.2033,11.8916,7.4200,-7.0000,0.0000,15.3890,9.5703,-17.3815,56.1791,-1.7423},
				{0.0200,31.0815,1.2534,8.4995,5.3170,2.3782,-50.2680,12.0372,7.2800,-7.0000,0.0000,15.5738,9.2428,-16.3766,50.2485,-1.7266},
				{0.0200,31.0603,1.1121,8.6065,5.3467,1.4818,-44.8225,12.1800,7.1400,-7.0000,0.0000,15.7525,8.9332,-15.4804,44.8070,-1.7123},
				{0.0200,31.0415,0.9734,8.7137,5.3603,0.6837,-39.9034,12.3200,7.0000,-7.0000,0.0000,15.9253,8.6396,-14.6826,39.8905,-1.6991},
				{0.0200,31.0248,0.8372,8.8209,5.3598,-0.0264,-35.5058,12.4572,6.8600,-7.0000,0.0000,16.0925,8.3601,-13.9727,35.4959,-1.6871},
				{0.0200,31.0099,0.7037,8.9278,5.3466,-0.6585,-31.6032,12.5916,6.7200,-7.0000,0.0000,16.2543,8.0933,-13.3408,31.5949,-1.6761},
				{0.0200,30.9968,0.5727,9.0343,5.3222,-1.2216,-28.1562,12.7232,6.5800,-7.0000,0.0000,16.4111,7.8377,-12.7778,28.1495,-1.6661},
				{0.0200,30.9851,0.4444,9.1400,5.2877,-1.7240,-25.1211,12.8520,6.4400,-7.0000,0.0000,16.5629,7.5922,-12.2755,25.1157,-1.6569},
				{0.0200,30.9748,0.3189,9.2449,5.2443,-2.1731,-22.4539,12.9780,6.3000,-7.0000,0.0000,16.7101,7.3557,-11.8265,22.4495,-1.6484},
				{0.0200,30.9657,0.1960,9.3488,5.1928,-2.5753,-20.1123,13.1012,6.1600,-7.0000,0.0000,16.8526,7.1272,-11.4243,20.1088,-1.6407},
				{0.0200,30.9578,0.0759,9.4514,5.1340,-2.9365,-18.0575,13.2216,6.0200,-7.0000,0.0000,16.9907,6.9059,-11.0633,18.0544,-1.6336},
				{0.0200,30.9508,-0.0415,9.5528,5.0688,-3.2615,-16.2540,13.3392,5.8800,-7.0000,0.0000,17.1245,6.6912,-10.7382,16.2515,-1.6271},
				{0.0200,30.9446,-0.1562,9.6528,4.9977,-3.5550,-14.6704,13.4540,5.7400,-7.0000,0.0000,17.2542,6.4823,-10.4449,14.6684,-1.6212},
				{0.0200,30.9393,-0.2680,9.7512,4.9213,-3.8205,-13.2789,13.5660,5.6000,-7.0000,0.0000,17.3798,6.2787,-10.1793,13.2773,-1.6157},
				{0.0200,30.9347,-0.3771,9.8480,4.8401,-4.0616,-12.0552,13.6752,5.4600,-7.0000,0.0000,17.5014,6.0799,-9.9382,12.0539,-1.6108},
				{0.0200,30.9307,-0.4835,9.9431,4.7544,-4.2812,-10.9780,13.7816,5.3200,-7.0000,0.0000,17.6191,5.8855,-9.7187,10.9769,-1.6063},
				{0.0200,30.9272,-0.5870,10.0364,4.6648,-4.4818,-10.0287,13.8852,5.1800,-7.0000,0.0000,17.7330,5.6952,-9.5181,10.0278,-1.6021},
				{0.0200,30.9242,-0.6878,10.1278,4.5715,-4.6656,-9.1912,13.9860,5.0400,-7.0000,0.0000,17.8431,5.5085,-9.3343,9.1902,-1.5984},
				{0.0200,30.9217,-0.7857,10.2173,4.4748,-4.8346,-8.4511,14.0840,4.9000,-7.0000,0.0000,17.9496,5.3252,-9.1653,8.4506,-1.5950},
				{0.0200,30.9196,-0.8809,10.3048,4.3750,-4.9906,-7.7967,14.1792,4.7600,-7.0000,0.0000,18.0525,5.1450,-9.0094,7.7960,-1.5919},
				{0.0200,30.9177,-0.9733,10.3902,4.2723,-5.1349,-7.2168,14.2716,4.6200,-7.0000,0.0000,18.1519,4.9677,-8.8651,7.2165,-1.5891},
				{0.0200,30.9162,-1.0629,10.4736,4.1669,-5.2689,-6.7025,14.3612,4.4800,-7.0000,0.0000,18.2478,4.7931,-8.7310,6.7022,-1.5866},
				{0.0200,30.9149,-1.1497,10.5548,4.0590,-5.3938,-6.2454,14.4480,4.3400,-7.0000,0.0000,18.3402,4.6210,-8.6061,6.2450,-1.5844},
				{0.0200,30.9139,-1.2336,10.6337,3.9488,-5.5106,-5.8382,14.5320,4.2000,-7.0000,0.0000,18.4292,4.4512,-8.4894,5.8380,-1.5824},
				{0.0200,30.9130,-1.3148,10.7105,3.8364,-5.6201,-5.4748,14.6132,4.0600,-7.0000,0.0000,18.5149,4.2836,-8.3799,5.4745,-1.5806},
				{0.0200,30.9123,-1.3932,10.7849,3.7220,-5.7231,-5.1494,14.6916,3.9200,-7.0000,0.0000,18.5972,4.1180,-8.2769,5.1492,-1.5790},
				{0.0200,30.9117,-1.4688,10.8570,3.6056,-5.8202,-4.8571,14.7672,3.7800,-7.0000,0.0000,18.6763,3.9544,-8.1798,4.8569,-1.5776},
				{0.0200,30.9113,-1.5416,10.9268,3.4873,-5.9121,-4.5933,14.8400,3.6400,-7.0000,0.0000,18.7522,3.7927,-8.0879,4.5932,-1.5764},
				{0.0200,30.9109,-1.6116,10.9941,3.3673,-5.9992,-4.3542,14.9100,3.5000,-7.0000,0.0000,18.8248,3.6327,-8.0008,4.3541,-1.5753},
				{0.0200,30.9107,-1.6788,11.0590,3.2457,-6.0819,-4.1361,14.9772,3.3600,-7.0000,0.0000,18.8943,3.4743,-7.9181,4.1361,-1.5744},
				{0.0200,30.9105,-1.7432,11.1215,3.1225,-6.1606,-3.9358,15.0416,3.2200,-7.0000,0.0000,18.9607,3.3175,-7.8394,3.9357,-1.5736},
				{0.0200,30.9103,-1.8048,11.1814,2.9978,-6.2356,-3.7504,15.1032,3.0800,-7.0000,0.0000,19.0239,3.1622,-7.7644,3.7504,-1.5730},
				{0.0200,30.9102,-1.8636,11.2389,2.8716,-6.3072,-3.5772,15.1620,2.9400,-7.0000,0.0000,19.0841,3.0084,-7.6928,3.5771,-1.5724},
				{0.0200,30.9101,-1.9196,11.2938,2.7441,-6.3755,-3.4137,15.2180,2.8000,-7.0000,0.0000,19.1412,2.8559,-7.6245,3.4137,-1.5720},
				{0.0200,30.9101,-1.9728,11.3461,2.6153,-6.4406,-3.2580,15.2712,2.6600,-7.0000,0.0000,19.1953,2.7047,-7.5594,3.2579,-1.5716},
				{0.0200,30.9100,-2.0232,11.3958,2.4852,-6.5028,-3.1078,15.3216,2.5200,-7.0000,0.0000,19.2464,2.5548,-7.4972,3.1078,-1.5713},
				{0.0200,30.9100,-2.0708,11.4428,2.3540,-6.5620,-2.9616,15.3692,2.3800,-7.0000,0.0000,19.2945,2.4060,-7.4380,2.9616,-1.5711},
				{0.0200,30.9100,-2.1156,11.4873,2.2216,-6.6184,-2.8177,15.4140,2.2400,-7.0000,0.0000,19.3397,2.2584,-7.3816,2.8177,-1.5710},
				{0.0200,30.9100,-2.1576,11.5290,2.0882,-6.6718,-2.6746,15.4560,2.1000,-7.0000,0.0000,19.3819,2.1118,-7.3282,2.6746,-1.5709},
				{0.0200,30.9100,-2.1968,11.5681,1.9538,-6.7225,-2.5312,15.4952,1.9600,-7.0000,0.0000,19.4212,1.9662,-7.2775,2.5312,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,1.6565,-14.8621,-406.9825,15.5284,1.8200,-7.0000,0.0000,19.4544,1.6598,-15.3201,-402.1300,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,-82.8256,-3398.1721,15.5284,1.6800,-7.0000,0.0000,19.4544,0.0000,-82.9922,-3383.6028,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,4141.2781,15.5284,1.5400,-7.0000,0.0000,19.4544,0.0000,0.0000,4149.6093,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,1.4000,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,1.2600,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,1.1200,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,0.9800,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,0.8400,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,0.7000,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,0.5600,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,0.4200,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,0.2800,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,0.1400,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,0.0000,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,11.6012,0.0000,0.0000,0.0000,15.5284,-0.1400,-7.0000,0.0000,19.4544,0.0000,0.0000,0.0000,-1.5708},

	    };

	@Override
	public double[][] getPath() {
	    return points;
	}
}