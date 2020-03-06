package frc.paths;

import com.team319.trajectory.Path;

public class SixBallp2 extends Path {
   // dt,x,y,left.pos,left.vel,left.acc,left.jerk,center.pos,center.vel,center.acc,center.jerk,right.pos,right.vel,right.acc,right.jerk,heading
	private static final double[][] points = {
				{0.0200,23.9968,-9.9724,0.0040,0.2000,10.0000,0.0000,0.0040,0.2000,10.0000,0.0000,0.0040,0.2000,10.0000,0.0000,-2.4938},
				{0.0200,23.9904,-9.9772,0.0120,0.4005,10.0269,1.3465,0.0120,0.4000,10.0000,0.0000,0.0120,0.3995,9.9731,-1.3465,-2.4938},
				{0.0200,23.9809,-9.9845,0.0240,0.6018,10.0637,1.8390,0.0240,0.6000,10.0000,0.0000,0.0240,0.5982,9.9363,-1.8390,-2.4938},
				{0.0200,23.9681,-9.9941,0.0401,0.8043,10.1234,2.9842,0.0400,0.8000,10.0000,0.0000,0.0399,0.7957,9.8766,-2.9842,-2.4939},
				{0.0200,23.9522,-10.0062,0.0603,1.0083,10.2020,3.9298,0.0600,1.0000,10.0000,0.0000,0.0597,0.9917,9.7980,-3.9298,-2.4941},
				{0.0200,23.9330,-10.0207,0.0846,1.2143,10.2987,4.8361,0.0840,1.2000,10.0000,0.0000,0.0834,1.1857,9.7013,-4.8361,-2.4943},
				{0.0200,23.9107,-10.0376,0.1130,1.4225,10.4127,5.6969,0.1120,1.4000,10.0000,0.0000,0.1110,1.3775,9.5873,-5.6970,-2.4947},
				{0.0200,23.8851,-10.0568,0.1457,1.6334,10.5428,6.5081,0.1440,1.6000,10.0000,0.0000,0.1423,1.5666,9.4572,-6.5082,-2.4954},
				{0.0200,23.8564,-10.0785,0.1826,1.8472,10.6882,7.2675,0.1800,1.8000,10.0000,0.0000,0.1774,1.7528,9.3118,-7.2676,-2.4962},
				{0.0200,23.8244,-10.1025,0.2239,2.0641,10.8477,7.9752,0.2200,2.0000,10.0000,0.0000,0.2161,1.9359,9.1523,-7.9753,-2.4974},
				{0.0200,23.7892,-10.1289,0.2696,2.2845,11.0203,8.6330,0.2640,2.2000,10.0000,0.0000,0.2584,2.1155,8.9797,-8.6332,-2.4990},
				{0.0200,23.7508,-10.1577,0.3198,2.5086,11.2052,9.2444,0.3120,2.4000,10.0000,0.0000,0.3042,2.2914,8.7948,-9.2448,-2.5010},
				{0.0200,23.7090,-10.1887,0.3745,2.7367,11.4015,9.8141,0.3640,2.6000,10.0000,0.0000,0.3535,2.4633,8.5985,-9.8147,-2.5035},
				{0.0200,23.6640,-10.2220,0.4339,2.9688,11.6084,10.3473,0.4200,2.8000,10.0000,0.0000,0.4061,2.6312,8.3915,-10.3481,-2.5066},
				{0.0200,23.6156,-10.2575,0.4980,3.2053,11.8254,10.8488,0.4800,3.0000,10.0000,0.0000,0.4620,2.7947,8.1745,-10.8499,-2.5104},
				{0.0200,23.5639,-10.2951,0.5669,3.4464,12.0519,11.3223,0.5440,3.2000,10.0000,0.0000,0.5211,2.9536,7.9480,-11.3238,-2.5149},
				{0.0200,23.5087,-10.3349,0.6408,3.6921,12.2873,11.7693,0.6120,3.4000,10.0000,0.0000,0.5832,3.1079,7.7126,-11.7713,-2.5203},
				{0.0200,23.4500,-10.3766,0.7196,3.9427,12.5310,12.1877,0.6840,3.6000,10.0000,0.0000,0.6484,3.2572,7.4688,-12.1903,-2.5266},
				{0.0200,23.3878,-10.4202,0.8036,4.1984,12.7824,12.5704,0.7600,3.8000,10.0000,0.0000,0.7164,3.4016,7.2173,-12.5737,-2.5340},
				{0.0200,23.3219,-10.4656,0.8928,4.4592,13.0405,12.9031,0.8400,4.0000,10.0000,0.0000,0.7872,3.5408,6.9592,-12.9074,-2.5424},
				{0.0200,23.2523,-10.5127,0.9873,4.7253,13.3037,13.1632,0.9240,4.2000,10.0000,0.0000,0.8607,3.6747,6.6958,-13.1685,-2.5521},
				{0.0200,23.1789,-10.5612,1.0872,4.9967,13.5701,13.3163,1.0120,4.4000,10.0000,0.0000,0.9368,3.8033,6.4294,-13.3228,-2.5631},
				{0.0200,23.1016,-10.6110,1.1927,5.2734,13.8364,13.3146,1.1040,4.6000,10.0000,0.0000,1.0153,3.9265,6.1629,-13.3224,-2.5755},
				{0.0200,23.0202,-10.6619,1.3038,5.5554,14.0982,13.0937,1.2000,4.8000,10.0000,0.0000,1.0962,4.0446,5.9008,-13.1032,-2.5895},
				{0.0200,22.9346,-10.7137,1.4206,5.8424,14.3497,12.5715,1.3000,5.0000,10.0000,0.0000,1.1793,4.1575,5.6492,-12.5825,-2.6050},
				{0.0200,22.8448,-10.7661,1.5433,6.1340,14.5826,11.6457,1.4040,5.2000,10.0000,0.0000,1.2647,4.2659,5.4160,-11.6584,-2.6222},
				{0.0200,22.7506,-10.8189,1.6719,6.4297,14.7865,10.1957,1.5120,5.4000,10.0000,0.0000,1.3521,4.3701,5.2118,-10.2099,-2.6412},
				{0.0200,22.6517,-10.8716,1.8065,6.7287,14.9482,8.0863,1.6240,5.6000,10.0000,0.0000,1.4415,4.4711,5.0498,-8.1018,-2.6620},
				{0.0200,22.5482,-10.9239,1.9471,7.0297,15.0517,5.1766,1.7400,5.8000,10.0000,0.0000,1.5329,4.5700,4.9459,-5.1928,-2.6847},
				{0.0200,22.4399,-10.9756,2.0937,7.3313,15.0785,1.3356,1.8600,6.0000,10.0000,0.0000,1.6263,4.6684,4.9189,-1.3517,-2.7092},
				{0.0200,22.3267,-11.0260,2.2463,7.6315,15.0078,-3.5346,1.9840,6.2000,10.0000,0.0000,1.7216,4.7682,4.9893,3.5199,-2.7356},
				{0.0200,22.2084,-11.0749,2.4049,7.9278,14.8184,-9.4682,2.1120,6.4000,10.0000,0.0000,1.8191,4.8717,5.1784,9.4565,-2.7638},
				{0.0200,22.0850,-11.1218,2.5693,8.2176,14.4904,-16.4003,2.2440,6.6000,10.0000,0.0000,1.9187,4.9819,5.5063,16.3935,-2.7936},
				{0.0200,21.9564,-11.1662,2.7392,8.4978,14.0077,-24.1331,2.3800,6.8000,10.0000,0.0000,2.0207,5.1016,5.9890,24.1332,-2.8249},
				{0.0200,21.8227,-11.2076,2.9145,8.7650,13.3614,-32.3150,2.5200,7.0000,10.0000,0.0000,2.1254,5.2344,6.6354,32.3239,-2.8574},
				{0.0200,21.6838,-11.2457,3.0948,9.0161,12.5526,-40.4436,2.6640,7.2000,10.0000,0.0000,2.2331,5.3833,7.4447,40.4624,-2.8909},
				{0.0200,21.5399,-11.2799,3.2798,9.2480,11.5945,-47.9040,2.8120,7.4000,10.0000,0.0000,2.3441,5.5513,8.4034,47.9335,-2.9250},
				{0.0200,21.3909,-11.3100,3.4690,9.4582,10.5136,-54.0465,2.9640,7.6000,10.0000,0.0000,2.4589,5.7410,9.4851,54.0856,-2.9592},
				{0.0200,21.2370,-11.3357,3.6619,9.6452,9.3477,-58.2902,3.1200,7.8000,10.0000,0.0000,2.5780,5.9541,10.6518,58.3369,-2.9933},
				{0.0200,21.0784,-11.3567,3.8580,9.8081,8.1431,-60.2323,3.2800,8.0000,10.0000,0.0000,2.7018,6.1912,11.8575,60.2832,-3.0266},
				{0.0200,20.9152,-11.3728,4.0570,9.9470,6.9485,-59.7300,3.4440,8.2000,10.0000,0.0000,2.8309,6.4523,13.0531,59.7811,-3.0588},
				{0.0200,20.7476,-11.3841,4.2582,10.0632,5.8099,-56.9293,3.6120,8.4000,10.0000,0.0000,2.9656,6.7361,14.1926,56.9764,-3.0895},
				{0.0200,20.5757,-11.3906,4.4614,10.1585,4.7653,-52.2299,3.7840,8.6000,10.0000,0.0000,3.1064,7.0409,15.2380,52.2693,-3.1182},
				{0.0200,20.3997,-11.3923,4.6661,10.2354,3.8414,-46.1981,3.9600,8.8000,10.0000,0.0000,3.2537,7.3641,16.1626,46.2272,-3.1447},
				{0.0200,20.2198,-11.3895,4.8720,10.2964,3.0523,-39.4530,4.1400,9.0000,10.0000,0.0000,3.4078,7.7032,16.9520,39.4703,-3.1686},
				{0.0200,20.0439,-11.3830,5.0700,9.8985,-19.8965,-1147.4388,4.3160,8.8000,-10.0000,0.0000,3.5618,7.7012,-0.0971,-852.4547,-3.1888},
				{0.0200,19.8722,-11.3734,5.2603,9.5131,-19.2681,31.4195,4.4880,8.6000,-10.0000,0.0000,3.7155,7.6867,-0.7270,-31.4946,-3.2057},
				{0.0200,19.7046,-11.3614,5.4431,9.1416,-18.5745,34.6787,4.6560,8.4000,-10.0000,0.0000,3.8687,7.6582,-1.4219,-34.7423,-3.2193},
				{0.0200,19.5411,-11.3478,5.6188,8.7845,-17.8557,35.9416,4.8200,8.2000,-10.0000,0.0000,4.0210,7.6154,-2.1417,-35.9934,-3.2301},
				{0.0200,19.3818,-11.3329,5.7876,8.4418,-17.1376,35.9029,4.9800,8.0000,-10.0000,0.0000,4.1722,7.5582,-2.8606,-35.9439,-3.2383},
				{0.0200,19.2266,-11.3174,5.9499,8.1130,-16.4361,35.0770,5.1360,7.8000,-10.0000,0.0000,4.3219,7.4869,-3.5628,-35.1084,-3.2440},
				{0.0200,19.0754,-11.3016,6.1058,7.7978,-15.7597,33.8196,5.2880,7.6000,-10.0000,0.0000,4.4699,7.4022,-4.2396,-33.8430,-3.2477},
				{0.0200,18.9283,-11.2857,6.2558,7.4956,-15.1125,32.3612,5.4360,7.4000,-10.0000,0.0000,4.6160,7.3044,-4.8872,-32.3779,-3.2494},
				{0.0200,18.7851,-11.2702,6.3999,7.2057,-14.4957,30.8380,5.5800,7.2000,-10.0000,0.0000,4.7599,7.1943,-5.5042,-30.8495,-3.2495},
				{0.0200,18.6459,-11.2552,6.5384,6.9275,-13.9093,29.3205,5.7200,7.0000,-10.0000,0.0000,4.9014,7.0725,-6.0908,-29.3278,-3.2482},
				{0.0200,18.5107,-11.2409,6.6716,6.6604,-13.3526,27.8343,5.8560,6.8000,-10.0000,0.0000,5.0402,6.9396,-6.6475,-27.8385,-3.2456},
				{0.0200,18.3794,-11.2274,6.7997,6.4039,-12.8251,26.3776,5.9880,6.6000,-10.0000,0.0000,5.1761,6.7961,-7.1751,-26.3794,-3.2420},
				{0.0200,18.2520,-11.2149,6.9229,6.1574,-12.3264,24.9330,6.1160,6.4000,-10.0000,0.0000,5.3089,6.6426,-7.6738,-24.9332,-3.2375},
				{0.0200,18.1285,-11.2033,7.0413,5.9203,-11.8569,23.4770,6.2400,6.2000,-10.0000,0.0000,5.4385,6.4797,-8.1433,-23.4760,-3.2324},
				{0.0200,18.0090,-11.1927,7.1551,5.6919,-11.4171,21.9859,6.3600,6.0000,-10.0000,0.0000,5.5647,6.3081,-8.5830,-21.9842,-3.2267},
				{0.0200,17.8934,-11.1832,7.2645,5.4718,-11.0083,20.4407,6.4760,5.8000,-10.0000,0.0000,5.6872,6.1282,-8.9918,-20.4386,-3.2207},
				{0.0200,17.7817,-11.1747,7.3697,5.2591,-10.6317,18.8292,6.5880,5.6000,-10.0000,0.0000,5.8061,5.9409,-9.3683,-18.8271,-3.2144},
				{0.0200,17.6740,-11.1672,7.4708,5.0534,-10.2888,17.1476,6.6960,5.4000,-10.0000,0.0000,5.9210,5.7466,-9.7112,-17.1457,-3.2080},
				{0.0200,17.5702,-11.1606,7.5679,4.8537,-9.9808,15.4007,6.8000,5.2000,-10.0000,0.0000,6.0319,5.5462,-10.0192,-15.3991,-3.2016},
				{0.0200,17.4703,-11.1550,7.6610,4.6596,-9.7088,13.6009,6.9000,5.0000,-10.0000,0.0000,6.1387,5.3404,-10.2912,-13.5996,-3.1953},
				{0.0200,17.3745,-11.1501,7.7504,4.4701,-9.4734,11.7668,6.9960,4.8000,-10.0000,0.0000,6.2413,5.1299,-10.5265,-11.7659,-3.1893},
				{0.0200,17.2826,-11.1460,7.8361,4.2846,-9.2750,9.9220,7.0880,4.6000,-10.0000,0.0000,6.3396,4.9154,-10.7249,-9.9214,-3.1834},
				{0.0200,17.1946,-11.1425,7.9182,4.1023,-9.1131,8.0925,7.1760,4.4000,-10.0000,0.0000,6.4336,4.6977,-10.8868,-8.0923,-3.1780},
				{0.0200,17.1107,-11.1397,7.9966,3.9226,-8.9870,6.3056,7.2600,4.2000,-10.0000,0.0000,6.5231,4.4774,-11.0129,-6.3057,-3.1728},
				{0.0200,17.0307,-11.1374,8.0715,3.7447,-8.8953,4.5879,7.3400,4.0000,-10.0000,0.0000,6.6082,4.2553,-11.1047,-4.5882,-3.1681},
				{0.0200,16.9547,-11.1355,8.1429,3.5680,-8.8360,2.9641,7.4160,3.8000,-10.0000,0.0000,6.6889,4.0320,-11.1639,-2.9645,-3.1639},
				{0.0200,16.8828,-11.1341,8.2107,3.3918,-8.8069,1.4560,7.4880,3.6000,-10.0000,0.0000,6.7650,3.8082,-11.1931,-1.4566,-3.1600},
				{0.0200,16.8148,-11.1329,8.2750,3.2157,-8.8052,0.0822,7.5560,3.4000,-10.0000,0.0000,6.8367,3.5843,-11.1947,-0.0827,-3.1566},
				{0.0200,16.7508,-11.1321,8.3358,3.0392,-8.8281,-1.1429,7.6200,3.2000,-10.0000,0.0000,6.9039,3.3608,-11.1719,1.1424,-3.1537},
				{0.0200,16.6908,-11.1314,8.3931,2.8617,-8.8722,-2.2084,7.6800,3.0000,-10.0000,0.0000,6.9667,3.1383,-11.1277,2.2079,-3.1511},
				{0.0200,16.6348,-11.1310,8.4467,2.6830,-8.9344,-3.1074,7.7360,2.8000,-10.0000,0.0000,7.0251,2.9170,-11.0656,3.1069,-3.1490},
				{0.0200,16.5828,-11.1306,8.4968,2.5028,-9.0111,-3.8363,7.7880,2.6000,-10.0000,0.0000,7.0790,2.6972,-10.9889,3.8359,-3.1472},
				{0.0200,16.5348,-11.1304,8.5432,2.3208,-9.0990,-4.3949,7.8360,2.4000,-10.0000,0.0000,7.1286,2.4792,-10.9010,4.3946,-3.1457},
				{0.0200,16.4908,-11.1302,8.5859,2.1369,-9.1947,-4.7861,7.8800,2.2000,-10.0000,0.0000,7.1738,2.2631,-10.8053,4.7858,-3.1445},
				{0.0200,16.4508,-11.1301,8.6250,1.9510,-9.2950,-5.0150,7.9200,2.0000,-10.0000,0.0000,7.2148,2.0490,-10.7050,5.0148,-3.1436},
				{0.0200,16.4148,-11.1301,8.6602,1.7631,-9.3968,-5.0892,7.9560,1.8000,-10.0000,0.0000,7.2516,1.8369,-10.6032,5.0891,-3.1430},
				{0.0200,16.3828,-11.1300,8.6917,1.5732,-9.4972,-5.0182,7.9880,1.6000,-10.0000,0.0000,7.2841,1.6268,-10.5028,5.0181,-3.1425},
				{0.0200,16.3548,-11.1300,8.7193,1.3813,-9.5934,-4.8130,8.0160,1.4000,-10.0000,0.0000,7.3125,1.4187,-10.4066,4.8130,-3.1421},
				{0.0200,16.3308,-11.1300,8.7431,1.1876,-9.6832,-4.4861,8.0400,1.2000,-10.0000,0.0000,7.3367,1.2124,-10.3168,4.4861,-3.1419},
				{0.0200,16.3108,-11.1300,8.7629,0.9923,-9.7642,-4.0508,8.0600,1.0000,-10.0000,0.0000,7.3569,1.0077,-10.2358,4.0508,-3.1417},
				{0.0200,16.2948,-11.1300,8.7788,0.7956,-9.8346,-3.5215,8.0760,0.8000,-10.0000,0.0000,7.3730,0.8044,-10.1654,3.5215,-3.1417},
				{0.0200,16.2828,-11.1300,8.7908,0.5978,-9.8929,-2.9131,8.0880,0.6000,-10.0000,0.0000,7.3850,0.6022,-10.1071,2.9131,-3.1416},
				{0.0200,16.2748,-11.1300,8.7988,0.3990,-9.9377,-2.2409,8.0960,0.4000,-10.0000,0.0000,7.3930,0.4010,-10.0623,2.2409,-3.1416},
				{0.0200,16.2708,-11.1300,8.8027,0.1997,-9.9681,-1.5207,8.1000,0.2000,-10.0000,0.0000,7.3970,0.2003,-10.0319,1.5207,-3.1416},
				{0.0200,16.2708,-11.1300,8.8027,0.0000,-9.9835,-0.7684,8.1000,-0.0000,-10.0000,0.0000,7.3970,0.0000,-10.0165,0.7684,-3.1416},

	    };

	@Override
	public double[][] getPath() {
	    return points;
	}
}