package frc.paths;

import com.team319.trajectory.Path;

public class RAR extends Path {
   // dt,x,y,left.pos,left.vel,left.acc,left.jerk,center.pos,center.vel,center.acc,center.jerk,right.pos,right.vel,right.acc,right.jerk,heading
	private static final double[][] points = {
				{0.0200,41.2683,6.0600,0.0017,0.0840,4.2000,0.0000,0.0017,0.0840,4.2000,0.0000,0.0017,0.0840,4.2000,0.0000,-3.1416},
				{0.0200,41.2650,6.0600,0.0050,0.1680,4.1996,-0.0179,0.0050,0.1680,4.2000,0.0000,0.0050,0.1680,4.2004,0.0179,-3.1416},
				{0.0200,41.2599,6.0600,0.0101,0.2520,4.1991,-0.0247,0.0101,0.2520,4.2000,0.0000,0.0101,0.2520,4.2009,0.0247,-3.1416},
				{0.0200,41.2532,6.0600,0.0168,0.3359,4.1983,-0.0404,0.0168,0.3360,4.2000,0.0000,0.0168,0.3361,4.2017,0.0404,-3.1416},
				{0.0200,41.2448,6.0600,0.0252,0.4199,4.1973,-0.0539,0.0252,0.4200,4.2000,0.0000,0.0252,0.4201,4.2027,0.0539,-3.1416},
				{0.0200,41.2347,6.0600,0.0353,0.5038,4.1959,-0.0674,0.0353,0.5040,4.2000,0.0000,0.0353,0.5042,4.2041,0.0674,-3.1416},
				{0.0200,41.2230,6.0600,0.0470,0.5877,4.1943,-0.0809,0.0470,0.5880,4.2000,0.0000,0.0471,0.5883,4.2057,0.0809,-3.1416},
				{0.0200,41.2095,6.0600,0.0605,0.6715,4.1924,-0.0945,0.0605,0.6720,4.2000,0.0000,0.0605,0.6725,4.2076,0.0945,-3.1416},
				{0.0200,41.1944,6.0600,0.0756,0.7553,4.1902,-0.1082,0.0756,0.7560,4.2000,0.0000,0.0756,0.7567,4.2098,0.1082,-3.1416},
				{0.0200,41.1776,6.0600,0.0923,0.8391,4.1878,-0.1219,0.0924,0.8400,4.2000,0.0000,0.0925,0.8409,4.2122,0.1219,-3.1415},
				{0.0200,41.1591,6.0600,0.1108,0.9228,4.1851,-0.1356,0.1109,0.9240,4.2000,0.0000,0.1110,0.9252,4.2149,0.1356,-3.1415},
				{0.0200,41.1390,6.0600,0.1309,1.0064,4.1821,-0.1495,0.1310,1.0080,4.2000,0.0000,0.1311,1.0096,4.2179,0.1495,-3.1415},
				{0.0200,41.1171,6.0600,0.1527,1.0900,4.1788,-0.1634,0.1529,1.0920,4.2000,0.0000,0.1530,1.0940,4.2212,0.1634,-3.1415},
				{0.0200,41.0936,6.0600,0.1762,1.1735,4.1753,-0.1774,0.1764,1.1760,4.2000,0.0000,0.1766,1.1785,4.2247,0.1774,-3.1414},
				{0.0200,41.0684,6.0600,0.2013,1.2570,4.1715,-0.1916,0.2016,1.2600,4.2000,0.0000,0.2019,1.2630,4.2285,0.1916,-3.1414},
				{0.0200,41.0415,6.0600,0.2281,1.3403,4.1673,-0.2059,0.2285,1.3440,4.2000,0.0000,0.2288,1.3477,4.2327,0.2059,-3.1413},
				{0.0200,41.0130,6.0600,0.2566,1.4236,4.1629,-0.2203,0.2570,1.4280,4.2000,0.0000,0.2575,1.4324,4.2371,0.2203,-3.1412},
				{0.0200,40.9827,6.0600,0.2868,1.5067,4.1582,-0.2350,0.2873,1.5120,4.2000,0.0000,0.2878,1.5173,4.2418,0.2350,-3.1411},
				{0.0200,40.9508,6.0599,0.3185,1.5898,4.1532,-0.2498,0.3192,1.5960,4.2000,0.0000,0.3199,1.6022,4.2468,0.2498,-3.1410},
				{0.0200,40.9172,6.0599,0.3520,1.6728,4.1479,-0.2648,0.3528,1.6800,4.2000,0.0000,0.3536,1.6872,4.2521,0.2648,-3.1409},
				{0.0200,40.8819,6.0599,0.3871,1.7556,4.1423,-0.2801,0.3881,1.7640,4.2000,0.0000,0.3890,1.7724,4.2577,0.2801,-3.1407},
				{0.0200,40.8450,6.0599,0.4239,1.8383,4.1364,-0.2956,0.4250,1.8480,4.2000,0.0000,0.4262,1.8577,4.2636,0.2956,-3.1405},
				{0.0200,40.8063,6.0598,0.4623,1.9209,4.1302,-0.3115,0.4637,1.9320,4.2000,0.0000,0.4651,1.9431,4.2698,0.3115,-3.1403},
				{0.0200,40.7660,6.0598,0.5024,2.0034,4.1236,-0.3276,0.5040,2.0160,4.2000,0.0000,0.5056,2.0286,4.2764,0.3276,-3.1401},
				{0.0200,40.7240,6.0597,0.5441,2.0857,4.1168,-0.3442,0.5460,2.1000,4.2000,0.0000,0.5479,2.1143,4.2832,0.3442,-3.1399},
				{0.0200,40.6803,6.0596,0.5874,2.1679,4.1095,-0.3612,0.5897,2.1840,4.2000,0.0000,0.5919,2.2001,4.2905,0.3611,-3.1396},
				{0.0200,40.6350,6.0595,0.6324,2.2500,4.1020,-0.3786,0.6350,2.2680,4.2000,0.0000,0.6376,2.2860,4.2980,0.3786,-3.1392},
				{0.0200,40.5879,6.0594,0.6791,2.3318,4.0940,-0.3965,0.6821,2.3520,4.2000,0.0000,0.6851,2.3722,4.3060,0.3965,-3.1389},
				{0.0200,40.5392,6.0592,0.7273,2.4136,4.0857,-0.4150,0.7308,2.4360,4.2000,0.0000,0.7343,2.4584,4.3143,0.4149,-3.1385},
				{0.0200,40.4888,6.0591,0.7773,2.4951,4.0771,-0.4340,0.7812,2.5200,4.2000,0.0000,0.7851,2.5449,4.3229,0.4340,-3.1380},
				{0.0200,40.4367,6.0589,0.8288,2.5765,4.0680,-0.4538,0.8333,2.6040,4.2000,0.0000,0.8378,2.6315,4.3320,0.4538,-3.1375},
				{0.0200,40.3830,6.0586,0.8819,2.6576,4.0585,-0.4743,0.8870,2.6880,4.2000,0.0000,0.8921,2.7184,4.3415,0.4743,-3.1370},
				{0.0200,40.3275,6.0584,0.9367,2.7386,4.0486,-0.4956,0.9425,2.7720,4.2000,0.0000,0.9483,2.8054,4.3514,0.4955,-3.1363},
				{0.0200,40.2704,6.0580,0.9931,2.8194,4.0382,-0.5177,0.9996,2.8560,4.2000,0.0000,1.0061,2.8926,4.3618,0.5178,-3.1357},
				{0.0200,40.2116,6.0577,1.0511,2.8999,4.0274,-0.5409,1.0584,2.9400,4.2000,0.0000,1.0657,2.9801,4.3726,0.5409,-3.1349},
				{0.0200,40.1511,6.0572,1.1107,2.9802,4.0161,-0.5651,1.1189,3.0240,4.2000,0.0000,1.1271,3.0678,4.3839,0.5651,-3.1342},
				{0.0200,40.0890,6.0568,1.1719,3.0603,4.0043,-0.5905,1.1810,3.1080,4.2000,0.0000,1.1902,3.1557,4.3957,0.5905,-3.1333},
				{0.0200,40.0251,6.0562,1.2347,3.1402,3.9920,-0.6172,1.2449,3.1920,4.2000,0.0000,1.2551,3.2438,4.4080,0.6172,-3.1323},
				{0.0200,39.9596,6.0556,1.2991,3.2197,3.9791,-0.6452,1.3104,3.2760,4.2000,0.0000,1.3217,3.3323,4.4209,0.6452,-3.1313},
				{0.0200,39.8924,6.0548,1.3651,3.2991,3.9656,-0.6748,1.3776,3.3600,4.2000,0.0000,1.3901,3.4209,4.4344,0.6748,-3.1302},
				{0.0200,39.8235,6.0540,1.4326,3.3781,3.9514,-0.7060,1.4465,3.4440,4.2000,0.0000,1.4603,3.5099,4.4486,0.7060,-3.1290},
				{0.0200,39.7530,6.0531,1.5018,3.4568,3.9367,-0.7390,1.5170,3.5280,4.2000,0.0000,1.5323,3.5992,4.4633,0.7390,-3.1277},
				{0.0200,39.6808,6.0520,1.5725,3.5352,3.9212,-0.7741,1.5893,3.6120,4.2000,0.0000,1.6061,3.6888,4.4788,0.7740,-3.1263},
				{0.0200,39.6068,6.0508,1.6448,3.6133,3.9049,-0.8112,1.6632,3.6960,4.2000,0.0000,1.6816,3.7787,4.4950,0.8112,-3.1248},
				{0.0200,39.5313,6.0495,1.7186,3.6911,3.8879,-0.8507,1.7388,3.7800,4.2000,0.0000,1.7590,3.8689,4.5121,0.8507,-3.1232},
				{0.0200,39.4540,6.0480,1.7939,3.7685,3.8701,-0.8928,1.8161,3.8640,4.2000,0.0000,1.8382,3.9595,4.5299,0.8927,-3.1215},
				{0.0200,39.3750,6.0464,1.8709,3.8455,3.8513,-0.9376,1.8950,3.9480,4.2000,0.0000,1.9192,4.0505,4.5487,0.9376,-3.1196},
				{0.0200,39.2944,6.0445,1.9493,3.9222,3.8316,-0.9855,1.9757,4.0320,4.2000,0.0000,2.0021,4.1418,4.5684,0.9853,-3.1176},
				{0.0200,39.2121,6.0424,2.0293,3.9984,3.8109,-1.0366,2.0580,4.1160,4.2000,0.0000,2.0867,4.2336,4.5891,1.0365,-3.1155},
				{0.0200,39.1282,6.0402,2.1107,4.0742,3.7891,-1.0913,2.1420,4.2000,4.2000,0.0000,2.1733,4.3258,4.6109,1.0912,-3.1132},
				{0.0200,39.0425,6.0376,2.1937,4.1495,3.7661,-1.1499,2.2277,4.2840,4.2000,0.0000,2.2616,4.4185,4.6339,1.1498,-3.1107},
				{0.0200,38.9552,6.0348,2.2782,4.2243,3.7418,-1.2128,2.3150,4.3680,4.2000,0.0000,2.3519,4.5117,4.6582,1.2126,-3.1081},
				{0.0200,38.8662,6.0317,2.3642,4.2986,3.7162,-1.2802,2.4041,4.4520,4.2000,0.0000,2.4440,4.6054,4.6838,1.2801,-3.1053},
				{0.0200,38.7756,6.0283,2.4516,4.3724,3.6891,-1.3527,2.4948,4.5360,4.2000,0.0000,2.5380,4.6996,4.7108,1.3525,-3.1024},
				{0.0200,38.6832,6.0245,2.5406,4.4456,3.6605,-1.4306,2.5872,4.6200,4.2000,0.0000,2.6338,4.7944,4.7394,1.4305,-3.0992},
				{0.0200,38.5893,6.0204,2.6309,4.5182,3.6302,-1.5145,2.6813,4.7040,4.2000,0.0000,2.7316,4.8898,4.7697,1.5142,-3.0958},
				{0.0200,38.4936,6.0158,2.7227,4.5902,3.5982,-1.6047,2.7770,4.7880,4.2000,0.0000,2.8314,4.9858,4.8018,1.6045,-3.0922},
				{0.0200,38.3963,6.0108,2.8160,4.6615,3.5641,-1.7019,2.8745,4.8720,4.2000,0.0000,2.9330,5.0825,4.8358,1.7016,-3.0884},
				{0.0200,38.2973,6.0054,2.9106,4.7320,3.5280,-1.8066,2.9736,4.9560,4.2000,0.0000,3.0366,5.1799,4.8720,1.8064,-3.0843},
				{0.0200,38.1967,5.9994,3.0066,4.8018,3.4896,-1.9195,3.0744,5.0400,4.2000,0.0000,3.1422,5.2782,4.9104,1.9192,-3.0800},
				{0.0200,38.0944,5.9928,3.1041,4.8708,3.4488,-2.0413,3.1769,5.1240,4.2000,0.0000,3.2497,5.3772,4.9512,2.0409,-3.0754},
				{0.0200,37.9905,5.9857,3.2028,4.9389,3.4053,-2.1725,3.2810,5.2080,4.2000,0.0000,3.3592,5.4771,4.9946,2.1721,-3.0705},
				{0.0200,37.8850,5.9779,3.3030,5.0061,3.3590,-2.3141,3.3869,5.2920,4.2000,0.0000,3.4708,5.5779,5.0409,2.3136,-3.0653},
				{0.0200,37.7778,5.9694,3.4044,5.0723,3.3097,-2.4666,3.4944,5.3760,4.2000,0.0000,3.5844,5.6797,5.0902,2.4661,-3.0598},
				{0.0200,37.6690,5.9602,3.5071,5.1374,3.2571,-2.6310,3.6036,5.4600,4.2000,0.0000,3.7001,5.7826,5.1428,2.6303,-3.0539},
				{0.0200,37.5585,5.9501,3.6112,5.2015,3.2009,-2.8079,3.7145,5.5440,4.2000,0.0000,3.8178,5.8865,5.1990,2.8073,-3.0477},
				{0.0200,37.4465,5.9392,3.7165,5.2643,3.1410,-2.9984,3.8270,5.6280,4.2000,0.0000,3.9376,5.9917,5.2589,2.9975,-3.0411},
				{0.0200,37.3329,5.9273,3.8230,5.3258,3.0769,-3.2029,3.9413,5.7120,4.2000,0.0000,4.0596,6.0982,5.3230,3.2021,-3.0340},
				{0.0200,37.2177,5.9145,3.9307,5.3860,3.0084,-3.4225,4.0572,5.7960,4.2000,0.0000,4.1837,6.2060,5.3914,3.4214,-3.0266},
				{0.0200,37.1009,5.9005,4.0396,5.4447,2.9353,-3.6574,4.1748,5.8800,4.2000,0.0000,4.3100,6.3153,5.4645,3.6563,-3.0187},
				{0.0200,36.9826,5.8854,4.1496,5.5018,2.8571,-3.9084,4.2941,5.9640,4.2000,0.0000,4.4385,6.4261,5.5427,3.9070,-3.0103},
				{0.0200,36.8628,5.8690,4.2608,5.5573,2.7736,-4.1754,4.4150,6.0480,4.2000,0.0000,4.5693,6.5387,5.6261,4.1738,-3.0014},
				{0.0200,36.7414,5.8513,4.3730,5.6110,2.6844,-4.4584,4.5377,6.1320,4.2000,0.0000,4.7024,6.6530,5.7153,4.4567,-2.9919},
				{0.0200,36.6186,5.8321,4.4862,5.6628,2.5893,-4.7570,4.6620,6.2160,4.2000,0.0000,4.8377,6.7692,5.8104,4.7548,-2.9818},
				{0.0200,36.4943,5.8114,4.6005,5.7125,2.4879,-5.0696,4.7880,6.3000,4.2000,0.0000,4.9755,6.8874,5.9117,5.0674,-2.9711},
				{0.0200,36.3686,5.7891,4.7157,5.7601,2.3800,-5.3947,4.9157,6.3840,4.2000,0.0000,5.1156,7.0078,6.0195,5.3920,-2.9598},
				{0.0200,36.2415,5.7649,4.8318,5.8054,2.2654,-5.7289,5.0450,6.4680,4.2000,0.0000,5.2583,7.1305,6.1341,5.7259,-2.9478},
				{0.0200,36.1131,5.7389,4.9488,5.8483,2.1441,-6.0680,5.1761,6.5520,4.2000,0.0000,5.4034,7.2556,6.2554,6.0645,-2.9350},
				{0.0200,35.9834,5.7108,5.0665,5.8886,2.0160,-6.4057,5.3088,6.6360,4.2000,0.0000,5.5510,7.3833,6.3834,6.4018,-2.9214},
				{0.0200,35.8524,5.6805,5.1851,5.9263,1.8813,-6.7339,5.4432,6.7200,4.2000,0.0000,5.7013,7.5136,6.5180,6.7295,-2.9069},
				{0.0200,35.7203,5.6478,5.3043,5.9611,1.7405,-7.0417,5.5793,6.8040,4.2000,0.0000,5.8542,7.6468,6.6587,7.0366,-2.8916},
				{0.0200,35.5871,5.6127,5.4242,5.9930,1.5942,-7.3147,5.7170,6.8880,4.2000,0.0000,6.0099,7.7829,6.8049,7.3091,-2.8753},
				{0.0200,35.4529,5.5748,5.5446,6.0218,1.4435,-7.5351,5.8565,6.9720,4.2000,0.0000,6.1683,7.9220,6.9555,7.5287,-2.8581},
				{0.0200,35.3189,5.5345,5.6646,5.9998,-1.1005,-127.1975,5.9965,7.0000,4.2000,0.0000,6.3283,8.0000,3.8995,-152.7978,-2.8399},
				{0.0200,35.1856,5.4916,5.7836,5.9522,-2.3818,-64.0669,6.1365,7.0000,4.2000,0.0000,6.4893,8.0476,2.3809,-75.9314,-2.8208},
				{0.0200,35.0532,5.4461,5.9017,5.9033,-2.4423,-3.0226,6.2765,7.0000,4.2000,0.0000,6.6512,8.0964,2.4413,3.0192,-2.8009},
				{0.0200,34.9217,5.3980,6.0188,5.8535,-2.4932,-2.5482,6.4165,7.0000,4.2000,0.0000,6.8141,8.1463,2.4922,2.5448,-2.7801},
				{0.0200,34.7913,5.3471,6.1348,5.8028,-2.5321,-1.9442,6.5565,7.0000,4.2000,0.0000,6.9781,8.1969,2.5310,1.9409,-2.7583},
				{0.0200,34.6621,5.2933,6.2499,5.7517,-2.5561,-1.1977,6.6965,7.0000,4.2000,0.0000,7.1430,8.2480,2.5549,1.1946,-2.7356},
				{0.0200,34.5341,5.2365,6.3639,5.7005,-2.5621,-0.2985,6.8365,7.0000,4.2000,0.0000,7.3090,8.2992,2.5608,0.2958,-2.7120},
				{0.0200,34.4076,5.1766,6.4769,5.6495,-2.5469,0.7594,6.9765,7.0000,4.2000,0.0000,7.4760,8.3501,2.5456,-0.7614,-2.6874},
				{0.0200,34.2825,5.1136,6.5888,5.5994,-2.5073,1.9760,7.1165,7.0000,4.2000,0.0000,7.6440,8.4002,2.5060,-1.9774,-2.6620},
				{0.0200,34.1592,5.0474,6.6999,5.5506,-2.4405,3.3439,7.2565,7.0000,4.2000,0.0000,7.8130,8.4490,2.4391,-3.3444,-2.6356},
				{0.0200,34.0377,4.9779,6.8099,5.5037,-2.3435,4.8462,7.3965,7.0000,4.2000,0.0000,7.9829,8.4959,2.3422,-4.8457,-2.6084},
				{0.0200,33.9181,4.9050,6.9191,5.4594,-2.2144,6.4554,7.5365,7.0000,4.2000,0.0000,8.1537,8.5401,2.2131,-6.4537,-2.5804},
				{0.0200,33.8007,4.8288,7.0275,5.4184,-2.0518,8.1327,7.6765,7.0000,4.2000,0.0000,8.3254,8.5811,2.0505,-8.1296,-2.5516},
				{0.0200,33.6855,4.7493,7.1351,5.3813,-1.8552,9.8280,7.8165,7.0000,4.2000,0.0000,8.4977,8.6182,1.8541,-9.8235,-2.5222},
				{0.0200,33.5727,4.6663,7.2421,5.3488,-1.6256,11.4819,7.9565,7.0000,4.2000,0.0000,8.6707,8.6507,1.6246,-11.4760,-2.4922},
				{0.0200,33.4625,4.5800,7.3485,5.3215,-1.3650,13.0283,8.0965,7.0000,4.2000,0.0000,8.8443,8.6780,1.3641,-13.0208,-2.4617},
				{0.0200,33.3550,4.4903,7.4545,5.2999,-1.0771,14.3980,8.2365,7.0000,4.2000,0.0000,9.0183,8.6995,1.0764,-14.3893,-2.4308},
				{0.0200,33.2504,4.3973,7.5602,5.2846,-0.7666,15.5250,8.3765,7.0000,4.2000,0.0000,9.1926,8.7148,0.7661,-15.5152,-2.3996},
				{0.0200,33.1487,4.3011,7.6657,5.2758,-0.4395,16.3511,8.5165,7.0000,4.2000,0.0000,9.3671,8.7236,0.4392,-16.3404,-2.3682},
				{0.0200,33.0501,4.2017,7.7712,5.2737,-0.1029,16.8316,8.6565,7.0000,4.2000,0.0000,9.5416,8.7257,0.1028,-16.8205,-2.3369},
				{0.0200,32.9546,4.0993,7.8768,5.2785,0.2359,16.9401,8.7965,7.0000,4.2000,0.0000,9.7160,8.7210,-0.2357,-16.9288,-2.3056},
				{0.0200,32.8624,3.9940,7.9826,5.2898,0.5693,16.6705,8.9365,7.0000,4.2000,0.0000,9.8902,8.7096,-0.5689,-16.6595,-2.2745},
				{0.0200,32.7734,3.8859,8.0887,5.3076,0.8901,16.0385,9.0765,7.0000,4.2000,0.0000,10.0640,8.6918,-0.8895,-16.0282,-2.2437},
				{0.0200,32.6879,3.7751,8.1953,5.3315,1.1917,15.0792,9.2165,7.0000,4.2000,0.0000,10.2374,8.6680,-1.1909,-15.0697,-2.2134},
				{0.0200,32.6057,3.6618,8.3026,5.3609,1.4685,13.8437,9.3565,7.0000,4.2000,0.0000,10.4101,8.6386,-1.4676,-13.8355,-2.1836},
				{0.0200,32.5268,3.5461,8.4105,5.3952,1.7164,12.3948,9.4965,7.0000,4.2000,0.0000,10.5822,8.6043,-1.7154,-12.3879,-2.1544},
				{0.0200,32.4514,3.4282,8.5191,5.4338,1.9324,10.8004,9.6365,7.0000,4.2000,0.0000,10.7535,8.5657,-1.9313,-10.7950,-2.1259},
				{0.0200,32.3792,3.3082,8.6287,5.4761,2.1150,9.1288,9.7765,7.0000,4.2000,0.0000,10.9240,8.5234,-2.1138,-9.1248,-2.0982},
				{0.0200,32.3104,3.1863,8.7391,5.5214,2.2639,7.4434,9.9165,7.0000,4.2000,0.0000,11.0936,8.4782,-2.2626,-7.4409,-2.0713},
				{0.0200,32.2448,3.0626,8.8505,5.5690,2.3799,5.7999,10.0565,7.0000,4.2000,0.0000,11.2622,8.4306,-2.3785,-5.7986,-2.0453},
				{0.0200,32.1825,2.9373,8.9628,5.6183,2.4647,4.2430,10.1965,7.0000,4.2000,0.0000,11.4298,8.3813,-2.4634,-4.2429,-2.0202},
				{0.0200,32.1232,2.8104,9.0762,5.6687,2.5209,2.8065,10.3365,7.0000,4.2000,0.0000,11.5964,8.3309,-2.5196,-2.8074,-1.9960},
				{0.0200,32.0676,2.6838,9.1892,5.6508,-0.8957,-170.8303,10.4748,6.9160,-4.2000,0.0000,11.7601,8.1809,-7.5024,-249.1436,-1.9730},
				{0.0200,32.0155,2.5575,9.3019,5.6312,-0.9792,-4.1716,10.6114,6.8320,-4.2000,0.0000,11.9207,8.0325,-7.4192,4.1636,-1.9512},
				{0.0200,31.9667,2.4316,9.4140,5.6096,-1.0796,-5.0237,10.7464,6.7480,-4.2000,0.0000,12.0784,7.8861,-7.3188,5.0159,-1.9305},
				{0.0200,31.9210,2.3064,9.5258,5.5858,-1.1924,-5.6369,10.8797,6.6640,-4.2000,0.0000,12.2333,7.7420,-7.2062,5.6294,-1.9109},
				{0.0200,31.8783,2.1820,9.6370,5.5595,-1.3133,-6.0473,11.0113,6.5800,-4.2000,0.0000,12.3853,7.6003,-7.0854,6.0402,-1.8923},
				{0.0200,31.8383,2.0584,9.7476,5.5307,-1.4391,-6.2892,11.1412,6.4960,-4.2000,0.0000,12.5345,7.4611,-6.9598,6.2827,-1.8748},
				{0.0200,31.8009,1.9357,9.8576,5.4994,-1.5670,-6.3946,11.2694,6.4120,-4.2000,0.0000,12.6810,7.3245,-6.8320,6.3886,-1.8582},
				{0.0200,31.7660,1.8140,9.9669,5.4655,-1.6948,-6.3917,11.3960,6.3280,-4.2000,0.0000,12.8248,7.1904,-6.7043,6.3862,-1.8425},
				{0.0200,31.7334,1.6935,10.0754,5.4291,-1.8209,-6.3049,11.5209,6.2440,-4.2000,0.0000,12.9660,7.0588,-6.5783,6.2999,-1.8277},
				{0.0200,31.7030,1.5741,10.1833,5.3902,-1.9440,-6.1550,11.6441,6.1600,-4.2000,0.0000,13.1046,6.9297,-6.4553,6.1506,-1.8137},
				{0.0200,31.6745,1.4560,10.2902,5.3489,-2.0632,-5.9592,11.7656,6.0760,-4.2000,0.0000,13.2406,6.8030,-6.3362,5.9552,-1.8005},
				{0.0200,31.6480,1.3391,10.3963,5.3054,-2.1778,-5.7315,11.8854,5.9920,-4.2000,0.0000,13.3742,6.6785,-6.2216,5.7280,-1.7880},
				{0.0200,31.6232,1.2236,10.5015,5.2596,-2.2875,-5.4832,12.0036,5.9080,-4.2000,0.0000,13.5053,6.5563,-6.1120,5.4801,-1.7762},
				{0.0200,31.6001,1.1094,10.6058,5.2118,-2.3920,-5.2233,12.1201,5.8240,-4.2000,0.0000,13.6340,6.4362,-6.0076,5.2206,-1.7651},
				{0.0200,31.5785,0.9967,10.7090,5.1620,-2.4911,-4.9587,12.2349,5.7400,-4.2000,0.0000,13.7604,6.3180,-5.9085,4.9561,-1.7546},
				{0.0200,31.5584,0.8853,10.8112,5.1103,-2.5850,-4.6945,12.3480,5.6560,-4.2000,0.0000,13.8844,6.2017,-5.8146,4.6924,-1.7446},
				{0.0200,31.5396,0.7755,10.9123,5.0568,-2.6737,-4.4350,12.4594,5.5720,-4.2000,0.0000,14.0062,6.0872,-5.7260,4.4330,-1.7353},
				{0.0200,31.5221,0.6671,11.0124,5.0016,-2.7574,-4.1827,12.5692,5.4880,-4.2000,0.0000,14.1257,5.9743,-5.6424,4.1810,-1.7264},
				{0.0200,31.5058,0.5603,11.1113,4.9449,-2.8362,-3.9399,12.6773,5.4040,-4.2000,0.0000,14.2429,5.8631,-5.5636,3.9384,-1.7181},
				{0.0200,31.4906,0.4550,11.2090,4.8867,-2.9103,-3.7077,12.7837,5.3200,-4.2000,0.0000,14.3580,5.7533,-5.4895,3.7064,-1.7102},
				{0.0200,31.4765,0.3512,11.3056,4.8271,-2.9801,-3.4872,12.8884,5.2360,-4.2000,0.0000,14.4709,5.6449,-5.4197,3.4860,-1.7028},
				{0.0200,31.4633,0.2490,11.4009,4.7662,-3.0457,-3.2786,12.9914,5.1520,-4.2000,0.0000,14.5816,5.5378,-5.3542,3.2776,-1.6958},
				{0.0200,31.4510,0.1484,11.4950,4.7040,-3.1073,-3.0821,13.0928,5.0680,-4.2000,0.0000,14.6903,5.4319,-5.2926,3.0812,-1.6891},
				{0.0200,31.4395,0.0494,11.5878,4.6407,-3.1653,-2.8976,13.1925,4.9840,-4.2000,0.0000,14.7968,5.3272,-5.2346,2.8967,-1.6829},
				{0.0200,31.4288,-0.0480,11.6793,4.5763,-3.2197,-2.7246,13.2905,4.9000,-4.2000,0.0000,14.9013,5.2236,-5.1801,2.7241,-1.6770},
				{0.0200,31.4189,-0.1438,11.7695,4.5109,-3.2710,-2.5631,13.3868,4.8160,-4.2000,0.0000,15.0037,5.1211,-5.1289,2.5625,-1.6715},
				{0.0200,31.4096,-0.2380,11.8584,4.4445,-3.3193,-2.4124,13.4814,4.7320,-4.2000,0.0000,15.1041,5.0194,-5.0807,2.4118,-1.6662},
				{0.0200,31.4010,-0.3306,11.9460,4.3772,-3.3647,-2.2719,13.5744,4.6480,-4.2000,0.0000,15.2025,4.9187,-5.0352,2.2714,-1.6613},
				{0.0200,31.3930,-0.4215,12.0321,4.3091,-3.4075,-2.1411,13.6657,4.5640,-4.2000,0.0000,15.2989,4.8189,-4.9924,2.1407,-1.6567},
				{0.0200,31.3855,-0.5108,12.1169,4.2401,-3.4479,-2.0195,13.7553,4.4800,-4.2000,0.0000,15.3933,4.7199,-4.9520,2.0191,-1.6523},
				{0.0200,31.3785,-0.5984,12.2003,4.1704,-3.4860,-1.9066,13.8432,4.3960,-4.2000,0.0000,15.4857,4.6216,-4.9139,1.9062,-1.6482},
				{0.0200,31.3720,-0.6844,12.2823,4.1000,-3.5221,-1.8015,13.9294,4.3120,-4.2000,0.0000,15.5762,4.5240,-4.8779,1.8013,-1.6444},
				{0.0200,31.3659,-0.7688,12.3629,4.0289,-3.5562,-1.7041,14.0140,4.2280,-4.2000,0.0000,15.6647,4.4271,-4.8438,1.7039,-1.6407},
				{0.0200,31.3603,-0.8514,12.4421,3.9571,-3.5884,-1.6137,14.0969,4.1440,-4.2000,0.0000,15.7513,4.3309,-4.8115,1.6134,-1.6373},
				{0.0200,31.3550,-0.9325,12.5198,3.8847,-3.6190,-1.5296,14.1781,4.0600,-4.2000,0.0000,15.8360,4.2353,-4.7810,1.5295,-1.6342},
				{0.0200,31.3501,-1.0118,12.5960,3.8117,-3.6481,-1.4517,14.2576,3.9760,-4.2000,0.0000,15.9188,4.1403,-4.7519,1.4514,-1.6312},
				{0.0200,31.3455,-1.0895,12.6708,3.7382,-3.6756,-1.3792,14.3354,3.8920,-4.2000,0.0000,15.9998,4.0458,-4.7243,1.3791,-1.6284},
				{0.0200,31.3412,-1.1656,12.7440,3.6642,-3.7019,-1.3120,14.4116,3.8080,-4.2000,0.0000,16.0788,3.9518,-4.6981,1.3118,-1.6258},
				{0.0200,31.3372,-1.2400,12.8158,3.5897,-3.7269,-1.2495,14.4861,3.7240,-4.2000,0.0000,16.1560,3.8583,-4.6731,1.2493,-1.6233},
				{0.0200,31.3335,-1.3127,12.8861,3.5146,-3.7507,-1.1913,14.5589,3.6400,-4.2000,0.0000,16.2313,3.7654,-4.6493,1.1912,-1.6210},
				{0.0200,31.3300,-1.3837,12.9549,3.4392,-3.7734,-1.1372,14.6300,3.5560,-4.2000,0.0000,16.3047,3.6728,-4.6266,1.1371,-1.6189},
				{0.0200,31.3267,-1.4531,13.0222,3.3633,-3.7952,-1.0868,14.6994,3.4720,-4.2000,0.0000,16.3763,3.5807,-4.6048,1.0867,-1.6169},
				{0.0200,31.3237,-1.5207,13.0879,3.2870,-3.8160,-1.0398,14.7672,3.3880,-4.2000,0.0000,16.4461,3.4890,-4.5840,1.0397,-1.6151},
				{0.0200,31.3208,-1.5868,13.1521,3.2102,-3.8359,-0.9959,14.8333,3.3040,-4.2000,0.0000,16.5141,3.3978,-4.5641,0.9958,-1.6134},
				{0.0200,31.3181,-1.6511,13.2148,3.1331,-3.8550,-0.9548,14.8977,3.2200,-4.2000,0.0000,16.5802,3.3069,-4.5450,0.9547,-1.6118},
				{0.0200,31.3156,-1.7138,13.2759,3.0557,-3.8733,-0.9163,14.9604,3.1360,-4.2000,0.0000,16.6445,3.2163,-4.5267,0.9162,-1.6104},
				{0.0200,31.3132,-1.7748,13.3355,2.9778,-3.8909,-0.8801,15.0214,3.0520,-4.2000,0.0000,16.7071,3.1262,-4.5091,0.8801,-1.6090},
				{0.0200,31.3110,-1.8341,13.3934,2.8997,-3.9078,-0.8462,15.0808,2.9680,-4.2000,0.0000,16.7678,3.0363,-4.4922,0.8461,-1.6078},
				{0.0200,31.3089,-1.8917,13.4499,2.8212,-3.9241,-0.8140,15.1385,2.8840,-4.2000,0.0000,16.8267,2.9468,-4.4759,0.8140,-1.6066},
				{0.0200,31.3069,-1.9477,13.5047,2.7424,-3.9398,-0.7837,15.1945,2.8000,-4.2000,0.0000,16.8839,2.8576,-4.4602,0.7836,-1.6056},
				{0.0200,31.3050,-2.0020,13.5580,2.6633,-3.9549,-0.7548,15.2488,2.7160,-4.2000,0.0000,16.9393,2.7687,-4.4451,0.7548,-1.6046},
				{0.0200,31.3033,-2.0546,13.6097,2.5839,-3.9694,-0.7273,15.3014,2.6320,-4.2000,0.0000,16.9929,2.6801,-4.4306,0.7274,-1.6037},
				{0.0200,31.3016,-2.1055,13.6598,2.5043,-3.9835,-0.7011,15.3524,2.5480,-4.2000,0.0000,17.0447,2.5917,-4.4165,0.7011,-1.6030},
				{0.0200,31.3001,-2.1548,13.7082,2.4243,-3.9970,-0.6759,15.4017,2.4640,-4.2000,0.0000,17.0948,2.5037,-4.4030,0.6760,-1.6022},
				{0.0200,31.2986,-2.2024,13.7551,2.3441,-4.0100,-0.6517,15.4493,2.3800,-4.2000,0.0000,17.1431,2.4159,-4.3900,0.6517,-1.6016},
				{0.0200,31.2972,-2.2483,13.8004,2.2637,-4.0226,-0.6282,15.4952,2.2960,-4.2000,0.0000,17.1896,2.3283,-4.3774,0.6282,-1.6010},
				{0.0200,31.2958,-2.2925,13.8441,2.1830,-4.0347,-0.6054,15.5394,2.2120,-4.2000,0.0000,17.2345,2.2410,-4.3653,0.6054,-1.6005},
				{0.0200,31.2946,-2.3350,13.8861,2.1020,-4.0463,-0.5832,15.5820,2.1280,-4.2000,0.0000,17.2775,2.1540,-4.3537,0.5832,-1.6000},
				{0.0200,31.2934,-2.3759,13.9265,2.0209,-4.0576,-0.5615,15.6229,2.0440,-4.2000,0.0000,17.3189,2.0671,-4.3424,0.5614,-1.5996},
				{0.0200,31.2923,-2.4151,13.9653,1.9395,-4.0684,-0.5401,15.6621,1.9600,-4.2000,0.0000,17.3585,1.9805,-4.3316,0.5401,-1.5992},
				{0.0200,31.2912,-2.4526,14.0025,1.8580,-4.0787,-0.5190,15.6996,1.8760,-4.2000,0.0000,17.3964,1.8940,-4.3212,0.5189,-1.5989},
				{0.0200,31.2902,-2.4884,14.0380,1.7762,-4.0887,-0.4980,15.7354,1.7920,-4.2000,0.0000,17.4325,1.8078,-4.3113,0.4980,-1.5986},
				{0.0200,31.2893,-2.5225,14.0719,1.6942,-4.0983,-0.4772,15.7696,1.7080,-4.2000,0.0000,17.4670,1.7218,-4.3017,0.4772,-1.5983},
				{0.0200,31.2884,-2.5550,14.1041,1.6121,-4.1074,-0.4564,15.8021,1.6240,-4.2000,0.0000,17.4997,1.6359,-4.2926,0.4564,-1.5981},
				{0.0200,31.2876,-2.5858,14.1347,1.5297,-4.1161,-0.4357,15.8329,1.5400,-4.2000,0.0000,17.5307,1.5503,-4.2839,0.4357,-1.5979},
				{0.0200,31.2868,-2.6149,14.1637,1.4473,-4.1244,-0.4148,15.8620,1.4560,-4.2000,0.0000,17.5600,1.4647,-4.2756,0.4148,-1.5978},
				{0.0200,31.2860,-2.6423,14.1909,1.3646,-4.1323,-0.3939,15.8894,1.3720,-4.2000,0.0000,17.5876,1.3794,-4.2677,0.3939,-1.5976},
				{0.0200,31.2853,-2.6681,14.2166,1.2818,-4.1397,-0.3729,15.9152,1.2880,-4.2000,0.0000,17.6135,1.2942,-4.2603,0.3728,-1.5975},
				{0.0200,31.2847,-2.6922,14.2406,1.1989,-4.1468,-0.3516,15.9393,1.2040,-4.2000,0.0000,17.6376,1.2091,-4.2532,0.3516,-1.5974},
				{0.0200,31.2841,-2.7146,14.2629,1.1158,-4.1534,-0.3302,15.9617,1.1200,-4.2000,0.0000,17.6601,1.1242,-4.2466,0.3302,-1.5974},
				{0.0200,31.2836,-2.7353,14.2835,1.0326,-4.1595,-0.3085,15.9824,1.0360,-4.2000,0.0000,17.6809,1.0394,-4.2405,0.3085,-1.5973},
				{0.0200,31.2830,-2.7543,14.3025,0.9493,-4.1653,-0.2866,16.0014,0.9520,-4.2000,0.0000,17.7000,0.9547,-4.2347,0.2866,-1.5972},
				{0.0200,31.2826,-2.7717,14.3198,0.8659,-4.1706,-0.2645,16.0188,0.8680,-4.2000,0.0000,17.7174,0.8701,-4.2294,0.2645,-1.5972},
				{0.0200,31.2822,-2.7873,14.3355,0.7824,-4.1754,-0.2421,16.0345,0.7840,-4.2000,0.0000,17.7331,0.7856,-4.2246,0.2421,-1.5972},
				{0.0200,31.2818,-2.8013,14.3495,0.6988,-4.1798,-0.2195,16.0485,0.7000,-4.2000,0.0000,17.7471,0.7012,-4.2202,0.2195,-1.5972},
				{0.0200,31.2815,-2.8136,14.3618,0.6151,-4.1837,-0.1967,16.0608,0.6160,-4.2000,0.0000,17.7595,0.6169,-4.2163,0.1967,-1.5971},
				{0.0200,31.2812,-2.8243,14.3724,0.5314,-4.1872,-0.1737,16.0714,0.5320,-4.2000,0.0000,17.7701,0.5326,-4.2128,0.1737,-1.5971},
				{0.0200,31.2810,-2.8332,14.3813,0.4476,-4.1902,-0.1504,16.0804,0.4480,-4.2000,0.0000,17.7791,0.4484,-4.2098,0.1504,-1.5971},
				{0.0200,31.2808,-2.8405,14.3886,0.3637,-4.1927,-0.1270,16.0877,0.3640,-4.2000,0.0000,17.7864,0.3643,-4.2073,0.1270,-1.5971},
				{0.0200,31.2806,-2.8461,14.3942,0.2798,-4.1948,-0.1034,16.0933,0.2800,-4.2000,0.0000,17.7920,0.2802,-4.2052,0.1034,-1.5971},
				{0.0200,31.2805,-2.8500,14.3981,0.1959,-4.1964,-0.0797,16.0972,0.1960,-4.2000,0.0000,17.7959,0.1961,-4.2036,0.0797,-1.5971},
				{0.0200,31.2805,-2.8523,14.4004,0.1120,-4.1975,-0.0558,16.0994,0.1120,-4.2000,0.0000,17.7982,0.1120,-4.2025,0.0558,-1.5971},
				{0.0200,31.2805,-2.8528,14.4009,0.0280,-4.1982,-0.0319,16.1000,0.0280,-4.2000,0.0000,17.7987,0.0280,-4.2018,0.0319,-1.5971},
				{0.0200,31.2805,-2.8517,14.4020,0.0560,1.3994,279.8794,16.0989,-0.0560,-4.2000,0.0000,17.7998,0.0560,1.4006,280.1206,-1.5971},

	    };

	@Override
	public double[][] getPath() {
	    return points;
	}
}