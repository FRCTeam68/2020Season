package frc.paths;

import com.team319.trajectory.Path;

public class Curve extends Path {
   // dt,x,y,left.pos,left.vel,left.acc,left.jerk,center.pos,center.vel,center.acc,center.jerk,right.pos,right.vel,right.acc,right.jerk,heading
	private static final double[][] points = {
				{0.0200,41.0760,4.8600,0.0040,0.2000,10.0000,0.0000,0.0040,0.2000,10.0000,0.0000,0.0040,0.2000,10.0000,0.0000,-3.1416},
				{0.0200,41.0680,4.8600,0.0120,0.4001,10.0044,0.2191,0.0120,0.4000,10.0000,0.0000,0.0120,0.3999,9.9956,-0.2191,-3.1416},
				{0.0200,41.0560,4.8600,0.0240,0.6003,10.0104,0.2985,0.0240,0.6000,10.0000,0.0000,0.0240,0.5997,9.9896,-0.2985,-3.1416},
				{0.0200,41.0400,4.8600,0.0400,0.8007,10.0200,0.4826,0.0400,0.8000,10.0000,0.0000,0.0400,0.7993,9.9800,-0.4826,-3.1416},
				{0.0200,41.0200,4.8600,0.0600,1.0013,10.0326,0.6318,0.0600,1.0000,10.0000,0.0000,0.0600,0.9987,9.9674,-0.6318,-3.1416},
				{0.0200,40.9960,4.8600,0.0841,1.2023,10.0481,0.7708,0.0840,1.2000,10.0000,0.0000,0.0839,1.1977,9.9519,-0.7708,-3.1417},
				{0.0200,40.9680,4.8600,0.1122,1.4036,10.0660,0.8973,0.1120,1.4000,10.0000,0.0000,0.1118,1.3964,9.9340,-0.8973,-3.1418},
				{0.0200,40.9360,4.8600,0.1443,1.6054,10.0862,1.0086,0.1440,1.6000,10.0000,0.0000,0.1437,1.5946,9.9138,-1.0086,-3.1419},
				{0.0200,40.9000,4.8600,0.1804,1.8075,10.1082,1.1023,0.1800,1.8000,10.0000,0.0000,0.1796,1.7925,9.8918,-1.1023,-3.1420},
				{0.0200,40.8600,4.8600,0.2206,2.0102,10.1317,1.1758,0.2200,2.0000,10.0000,0.0000,0.2194,1.9898,9.8683,-1.1758,-3.1422},
				{0.0200,40.8160,4.8601,0.2649,2.2133,10.1563,1.2268,0.2640,2.2000,10.0000,0.0000,0.2631,2.1867,9.8437,-1.2268,-3.1425},
				{0.0200,40.7680,4.8601,0.3132,2.4169,10.1813,1.2530,0.3120,2.4000,10.0000,0.0000,0.3108,2.3831,9.8187,-1.2530,-3.1428},
				{0.0200,40.7160,4.8602,0.3657,2.6210,10.2064,1.2523,0.3640,2.6000,10.0000,0.0000,0.3623,2.5790,9.7936,-1.2523,-3.1432},
				{0.0200,40.6600,4.8603,0.4222,2.8256,10.2308,1.2229,0.4200,2.8000,10.0000,0.0000,0.4178,2.7744,9.7692,-1.2229,-3.1438},
				{0.0200,40.6000,4.8605,0.4828,3.0307,10.2541,1.1631,0.4800,3.0000,10.0000,0.0000,0.4772,2.9693,9.7459,-1.1631,-3.1444},
				{0.0200,40.5360,4.8607,0.5475,3.2362,10.2755,1.0715,0.5440,3.2000,10.0000,0.0000,0.5405,3.1638,9.7245,-1.0716,-3.1451},
				{0.0200,40.4680,4.8609,0.6163,3.4421,10.2945,0.9475,0.6120,3.4000,10.0000,0.0000,0.6077,3.3579,9.7055,-0.9475,-3.1459},
				{0.0200,40.3960,4.8613,0.6893,3.6483,10.3103,0.7904,0.6840,3.6000,10.0000,0.0000,0.6787,3.5517,9.6897,-0.7904,-3.1469},
				{0.0200,40.3200,4.8617,0.7664,3.8548,10.3223,0.6002,0.7600,3.8000,10.0000,0.0000,0.7536,3.7452,9.6777,-0.6003,-3.1480},
				{0.0200,40.2400,4.8623,0.8476,4.0614,10.3298,0.3776,0.8400,4.0000,10.0000,0.0000,0.8324,3.9386,9.6702,-0.3776,-3.1492},
				{0.0200,40.1560,4.8630,0.9330,4.2680,10.3323,0.1234,0.9240,4.2000,10.0000,0.0000,0.9150,4.1320,9.6677,-0.1234,-3.1506},
				{0.0200,40.0680,4.8638,1.0225,4.4746,10.3291,-0.1608,1.0120,4.4000,10.0000,0.0000,1.0015,4.3254,9.6709,0.1607,-3.1521},
				{0.0200,39.9760,4.8649,1.1161,4.6810,10.3196,-0.4730,1.1040,4.6000,10.0000,0.0000,1.0919,4.5190,9.6804,0.4730,-3.1537},
				{0.0200,39.8800,4.8661,1.2139,4.8871,10.3034,-0.8108,1.2000,4.8000,10.0000,0.0000,1.1861,4.7129,9.6966,0.8108,-3.1554},
				{0.0200,39.7800,4.8676,1.3157,5.0927,10.2800,-1.1716,1.3000,5.0000,10.0000,0.0000,1.2843,4.9073,9.7200,1.1715,-3.1573},
				{0.0200,39.6761,4.8693,1.4217,5.2976,10.2489,-1.5522,1.4040,5.2000,10.0000,0.0000,1.3863,5.1024,9.7510,1.5523,-3.1592},
				{0.0200,39.5681,4.8713,1.5317,5.5018,10.2099,-1.9499,1.5120,5.4000,10.0000,0.0000,1.4923,5.2982,9.7900,1.9499,-3.1613},
				{0.0200,39.4561,4.8737,1.6458,5.7051,10.1627,-2.3615,1.6240,5.6000,10.0000,0.0000,1.6022,5.4949,9.8373,2.3615,-3.1634},
				{0.0200,39.3401,4.8763,1.7639,5.9072,10.1070,-2.7843,1.7400,5.8000,10.0000,0.0000,1.7161,5.6928,9.8930,2.7844,-3.1655},
				{0.0200,39.2202,4.8793,1.8861,6.1081,10.0427,-3.2162,1.8600,6.0000,10.0000,0.0000,1.8339,5.8919,9.9573,3.2163,-3.1677},
				{0.0200,39.0962,4.8827,2.0123,6.3075,9.9696,-3.6553,1.9840,6.2000,10.0000,0.0000,1.9557,6.0925,10.0304,3.6555,-3.1698},
				{0.0200,38.9683,4.8865,2.1424,6.5052,9.8876,-4.1011,2.1120,6.4000,10.0000,0.0000,2.0816,6.2948,10.1124,4.1013,-3.1719},
				{0.0200,38.8363,4.8906,2.2764,6.7012,9.7965,-4.5536,2.2440,6.6000,10.0000,0.0000,2.2116,6.4988,10.2035,4.5539,-3.1740},
				{0.0200,38.7004,4.8951,2.4143,6.8951,9.6962,-5.0146,2.3800,6.8000,10.0000,0.0000,2.3457,6.7049,10.3038,5.0148,-3.1759},
				{0.0200,38.5605,4.9001,2.5560,7.0868,9.5865,-5.4868,2.5200,7.0000,10.0000,0.0000,2.4840,6.9132,10.4135,5.4871,-3.1776},
				{0.0200,38.4166,4.9053,2.7015,7.2762,9.4670,-5.9753,2.6640,7.2000,10.0000,0.0000,2.6265,7.1238,10.5331,5.9755,-3.1791},
				{0.0200,38.2687,4.9110,2.8508,7.4629,9.3372,-6.4865,2.8120,7.4000,10.0000,0.0000,2.7732,7.3371,10.6628,6.4866,-3.1804},
				{0.0200,38.1168,4.9170,3.0037,7.6468,9.1967,-7.0294,2.9640,7.6000,10.0000,0.0000,2.9243,7.5532,10.8034,7.0294,-3.1813},
				{0.0200,37.9609,4.9232,3.1603,7.8277,9.0443,-7.6152,3.1200,7.8000,10.0000,0.0000,3.0797,7.7723,10.9557,7.6151,-3.1819},
				{0.0200,37.8011,4.9297,3.3204,8.0053,8.8792,-8.2583,3.2800,8.0000,10.0000,0.0000,3.2396,7.9947,11.1208,8.2579,-3.1820},
				{0.0200,37.6372,4.9363,3.4840,8.1793,8.6997,-8.9759,3.4440,8.2000,10.0000,0.0000,3.4040,8.2207,11.3003,8.9752,-3.1816},
				{0.0200,37.4693,4.9429,3.6510,8.3494,8.5039,-9.7893,3.6120,8.4000,10.0000,0.0000,3.5730,8.4506,11.4961,9.7880,-3.1806},
				{0.0200,37.2975,4.9495,3.8213,8.5152,8.2894,-10.7238,3.7840,8.6000,10.0000,0.0000,3.7467,8.6848,11.7105,10.7219,-3.1789},
				{0.0200,37.1216,4.9558,3.9948,8.6762,8.0532,-11.8101,3.9600,8.8000,10.0000,0.0000,3.9252,8.9238,11.9467,11.8074,-3.1764},
				{0.0200,36.9417,4.9618,4.1714,8.8321,7.7915,-13.0851,4.1400,9.0000,10.0000,0.0000,4.1086,9.1679,12.2083,13.0813,-3.1730},
				{0.0200,36.7578,4.9672,4.3511,8.9821,7.4996,-14.5929,4.3240,9.2000,10.0000,0.0000,4.2969,9.4179,12.5001,14.5876,-3.1687},
				{0.0200,36.5698,4.9718,4.5336,9.1255,7.1719,-16.3863,4.5120,9.4000,10.0000,0.0000,4.4904,9.6745,12.8276,16.3789,-3.1632},
				{0.0200,36.3778,4.9753,4.7188,9.2615,6.8013,-18.5284,4.7040,9.6000,10.0000,0.0000,4.6892,9.9384,13.1980,18.5184,-3.1564},
				{0.0200,36.1819,4.9774,4.9066,9.3891,6.3795,-21.0940,4.9000,9.8000,10.0000,0.0000,4.8934,10.2108,13.6196,21.0805,-3.1482},
				{0.0200,35.9819,4.9778,5.0967,9.5070,5.8960,-24.1712,5.1000,10.0000,10.0000,0.0000,5.1033,10.4929,14.1027,24.1528,-3.1383},
				{0.0200,35.7819,4.9760,5.2853,9.4262,-4.0436,-496.9801,5.3000,10.0000,10.0000,0.0000,5.3147,10.5737,4.0421,-503.0272,-3.1269},
				{0.0200,35.5819,4.9718,5.4720,9.3380,-4.4068,-18.1616,5.5000,10.0000,10.0000,0.0000,5.5280,10.6618,4.4050,18.1428,-3.1136},
				{0.0200,35.3820,4.9647,5.6569,9.2417,-4.8147,-20.3929,5.7000,10.0000,10.0000,0.0000,5.7431,10.7581,4.8124,20.3696,-3.0985},
				{0.0200,35.1823,4.9544,5.8396,9.1363,-5.2719,-22.8619,5.9000,10.0000,10.0000,0.0000,5.9604,10.8635,5.2690,22.8334,-3.0812},
				{0.0200,34.9828,4.9404,6.0200,9.0206,-5.7824,-25.5245,6.1000,10.0000,10.0000,0.0000,6.1800,10.9790,5.7788,25.4895,-3.0616},
				{0.0200,34.7836,4.9223,6.1979,8.8937,-6.3482,-28.2925,6.3000,10.0000,10.0000,0.0000,6.4021,11.1059,6.3438,28.2495,-3.0395},
				{0.0200,34.5849,4.8995,6.3730,8.7543,-6.9684,-31.0096,6.5000,10.0000,10.0000,0.0000,6.6270,11.2452,6.9630,30.9572,-3.0146},
				{0.0200,34.3869,4.8714,6.5450,8.6016,-7.6368,-33.4196,6.7000,10.0000,10.0000,0.0000,6.8550,11.3978,7.6301,33.3563,-2.9866},
				{0.0200,34.1898,4.8375,6.7137,8.4348,-8.3393,-35.1256,6.9000,10.0000,10.0000,0.0000,7.0862,11.5644,8.3311,35.0504,-2.9553},
				{0.0200,33.9940,4.7971,6.8788,8.2538,-9.0502,-35.5449,7.1000,10.0000,10.0000,0.0000,7.3211,11.7452,9.0402,35.4572,-2.9204},
				{0.0200,33.7997,4.7495,7.0399,8.0592,-9.7276,-33.8703,7.3000,10.0000,10.0000,0.0000,7.5599,11.9395,9.7157,33.7722,-2.8816},
				{0.0200,33.6076,4.6940,7.1970,7.8531,-10.3089,-29.0655,7.5000,10.0000,10.0000,0.0000,7.8028,12.1454,10.2949,28.9608,-2.8387},
				{0.0200,33.4181,4.6300,7.3498,7.6389,-10.7076,-19.9319,7.7000,10.0000,10.0000,0.0000,8.0500,12.3592,10.6915,19.8298,-2.7915},
				{0.0200,33.2321,4.5566,7.4982,7.4226,-10.8139,-5.3141,7.9000,10.0000,10.0000,0.0000,8.3015,12.5752,10.7961,5.2284,-2.7399},
				{0.0200,33.0503,4.4734,7.6425,7.2125,-10.5037,15.5093,8.1000,10.0000,10.0000,0.0000,8.5572,12.7849,10.4849,-15.5586,-2.6842},
				{0.0200,32.8735,4.3798,7.7829,7.0194,-9.6587,42.2512,8.3000,10.0000,10.0000,0.0000,8.8168,12.9777,9.6401,-42.2407,-2.6246},
				{0.0200,32.7029,4.2756,7.9200,6.8554,-8.1985,73.0098,8.5000,10.0000,10.0000,0.0000,9.0796,13.1413,8.1817,-72.9184,-2.5617},
				{0.0200,32.5392,4.1607,8.0546,6.7330,-6.1190,103.9724,8.7000,10.0000,10.0000,0.0000,9.3449,13.2634,6.1059,-103.7894,-2.4964},
				{0.0200,32.3836,4.0352,8.1879,6.6626,-3.5219,129.8574,8.9000,10.0000,10.0000,0.0000,9.6115,13.3337,3.5141,-129.5911,-2.4297},
				{0.0200,32.2366,3.8996,8.3209,6.6503,-0.6161,145.2884,9.1000,10.0000,10.0000,0.0000,9.8785,13.3460,0.6147,-144.9691,-2.3627},
				{0.0200,32.0990,3.7544,8.4548,6.6966,2.3178,146.6942,9.3000,10.0000,10.0000,0.0000,10.1445,13.2997,-2.3126,-146.3683,-2.2967},
				{0.0200,31.9712,3.6007,8.5908,6.7965,4.9932,133.7692,9.5000,10.0000,10.0000,0.0000,10.4085,13.2001,-4.9823,-133.4846,-2.2326},
				{0.0200,31.8532,3.4392,8.7296,6.9402,7.1852,109.5989,9.7000,10.0000,10.0000,0.0000,10.6696,13.0567,-7.1701,-109.3903,-2.1715},
				{0.0200,31.7451,3.2710,8.8719,7.1156,8.7726,79.3724,9.9000,10.0000,10.0000,0.0000,10.9272,12.8816,-8.7552,-79.2536,-2.1138},
				{0.0200,31.6464,3.0971,9.0181,7.3105,9.7423,48.4843,10.1000,10.0000,10.0000,0.0000,11.1810,12.6871,-9.7242,-48.4485,-2.0600},
				{0.0200,31.5569,2.9182,9.1684,7.5137,10.1630,21.0339,10.3000,10.0000,10.0000,0.0000,11.4307,12.4842,-10.1454,-21.0625,-2.0103},
				{0.0200,31.4760,2.7353,9.3227,7.7167,10.1476,-0.7664,10.5000,10.0000,10.0000,0.0000,11.6763,12.2816,-10.1315,0.6962,-1.9647},
				{0.0200,31.4032,2.5491,9.4810,7.9131,9.8198,-16.3895,10.7000,10.0000,10.0000,0.0000,11.9180,12.0855,-9.8056,16.2981,-1.9229},
				{0.0200,31.3391,2.3639,9.6397,7.9352,1.1065,-435.6656,10.8960,9.8000,-10.0000,0.0000,12.1513,11.6636,-21.0908,-564.2604,-1.8856},
				{0.0200,31.2828,2.1803,9.7984,7.9348,-0.0222,-56.4391,11.0880,9.6000,-10.0000,0.0000,12.3766,11.2643,-19.9653,56.2711,-1.8523},
				{0.0200,31.2333,1.9990,9.9566,7.9126,-1.1070,-54.2380,11.2760,9.4000,-10.0000,0.0000,12.5943,10.8867,-18.8833,54.1022,-1.8226},
				{0.0200,31.1898,1.8202,10.1140,7.8705,-2.1095,-50.1271,11.4600,9.2000,-10.0000,0.0000,12.8049,10.5290,-17.8829,50.0194,-1.7960},
				{0.0200,31.1517,1.6443,10.2702,7.8102,-3.0137,-45.2069,11.6400,9.0000,-10.0000,0.0000,13.0087,10.1894,-16.9805,45.1226,-1.7722},
				{0.0200,31.1184,1.4715,10.4249,7.7338,-3.8165,-40.1423,11.8160,8.8000,-10.0000,0.0000,13.2060,9.8658,-16.1789,40.0773,-1.7509},
				{0.0200,31.0892,1.3019,10.5778,7.6434,-4.5226,-35.3039,11.9880,8.6000,-10.0000,0.0000,13.3971,9.5563,-15.4738,35.2534,-1.7318},
				{0.0200,31.0638,1.1359,10.7286,7.5406,-5.1401,-30.8737,12.1560,8.4000,-10.0000,0.0000,13.5823,9.2592,-14.8571,30.8348,-1.7146},
				{0.0200,31.0415,0.9734,10.8771,7.4270,-5.6785,-26.9218,12.3200,8.2000,-10.0000,0.0000,13.7617,8.9728,-14.3193,26.8916,-1.6991},
				{0.0200,31.0222,0.8146,11.0232,7.3041,-6.1476,-23.4537,12.4800,8.0000,-10.0000,0.0000,13.9357,8.6958,-13.8507,23.4306,-1.6852},
				{0.0200,31.0053,0.6595,11.1667,7.1729,-6.5564,-20.4423,12.6360,7.8000,-10.0000,0.0000,14.1042,8.4270,-13.4422,20.4241,-1.6727},
				{0.0200,30.9907,0.5082,11.3074,7.0347,-6.9133,-17.8442,12.7880,7.6000,-10.0000,0.0000,14.2675,8.1652,-13.0856,17.8299,-1.6614},
				{0.0200,30.9781,0.3607,11.4452,6.8902,-7.2255,-15.6110,12.9360,7.4000,-10.0000,0.0000,14.4257,7.9098,-12.7736,15.6002,-1.6512},
				{0.0200,30.9672,0.2172,11.5800,6.7402,-7.4995,-13.6961,13.0800,7.2000,-10.0000,0.0000,14.5789,7.6598,-12.4999,13.6872,-1.6420},
				{0.0200,30.9579,0.0775,11.7117,6.5854,-7.7406,-12.0547,13.2200,7.0000,-10.0000,0.0000,14.7272,7.4146,-12.2589,12.0480,-1.6337},
				{0.0200,30.9498,-0.0583,11.8402,6.4263,-7.9535,-10.6482,13.3560,6.8000,-10.0000,0.0000,14.8707,7.1737,-12.0461,10.6426,-1.6262},
				{0.0200,30.9430,-0.1901,11.9655,6.2634,-8.1424,-9.4416,13.4880,6.6000,-10.0000,0.0000,15.0094,6.9365,-11.8573,9.4376,-1.6195},
				{0.0200,30.9371,-0.3180,12.0874,6.0972,-8.3105,-8.4060,13.6160,6.4000,-10.0000,0.0000,15.1434,6.7027,-11.6893,8.4023,-1.6134},
				{0.0200,30.9322,-0.4419,12.2060,5.9280,-8.4608,-7.5151,13.7400,6.2000,-10.0000,0.0000,15.2729,6.4720,-11.5390,7.5126,-1.6080},
				{0.0200,30.9280,-0.5618,12.3211,5.7561,-8.5957,-6.7482,13.8600,6.0000,-10.0000,0.0000,15.3978,6.2439,-11.4041,6.7459,-1.6031},
				{0.0200,30.9245,-0.6778,12.4327,5.5818,-8.7175,-6.0863,13.9760,5.8000,-10.0000,0.0000,15.5181,6.0182,-11.2824,6.0846,-1.5987},
				{0.0200,30.9216,-0.7897,12.5408,5.4052,-8.8277,-5.5141,14.0880,5.6000,-10.0000,0.0000,15.6340,5.7948,-11.1721,5.5127,-1.5948},
				{0.0200,30.9192,-0.8977,12.6454,5.2266,-8.9281,-5.0183,14.1960,5.4000,-10.0000,0.0000,15.7455,5.5734,-11.0718,5.0171,-1.5914},
				{0.0200,30.9172,-1.0017,12.7463,5.0462,-9.0199,-4.5875,14.3000,5.2000,-10.0000,0.0000,15.8526,5.3538,-10.9801,4.5866,-1.5883},
				{0.0200,30.9156,-1.1017,12.8436,4.8642,-9.1041,-4.2123,14.4000,5.0000,-10.0000,0.0000,15.9553,5.1358,-10.8958,4.2115,-1.5856},
				{0.0200,30.9143,-1.1977,12.9372,4.6805,-9.1818,-3.8842,14.4960,4.8000,-10.0000,0.0000,16.0537,4.9195,-10.8182,3.8836,-1.5832},
				{0.0200,30.9133,-1.2896,13.0271,4.4955,-9.2537,-3.5962,14.5880,4.6000,-10.0000,0.0000,16.1478,4.7045,-10.7463,3.5958,-1.5811},
				{0.0200,30.9124,-1.3776,13.1133,4.3090,-9.3206,-3.3422,14.6760,4.4000,-10.0000,0.0000,16.2376,4.4910,-10.6794,3.3419,-1.5793},
				{0.0200,30.9118,-1.4616,13.1957,4.1214,-9.3829,-3.1169,14.7600,4.2000,-10.0000,0.0000,16.3232,4.2786,-10.6171,3.1165,-1.5777},
				{0.0200,30.9113,-1.5416,13.2744,3.9326,-9.4412,-2.9153,14.8400,4.0000,-10.0000,0.0000,16.4045,4.0674,-10.5588,2.9152,-1.5764},
				{0.0200,30.9109,-1.6176,13.3492,3.7426,-9.4959,-2.7337,14.9160,3.8000,-10.0000,0.0000,16.4816,3.8574,-10.5041,2.7335,-1.5752},
				{0.0200,30.9106,-1.6896,13.4202,3.5517,-9.5472,-2.5682,14.9880,3.6000,-10.0000,0.0000,16.5546,3.6483,-10.4528,2.5681,-1.5743},
				{0.0200,30.9104,-1.7576,13.4874,3.3598,-9.5956,-2.4158,15.0560,3.4000,-10.0000,0.0000,16.6234,3.4402,-10.4044,2.4157,-1.5735},
				{0.0200,30.9103,-1.8216,13.5508,3.1670,-9.6410,-2.2736,15.1200,3.2000,-10.0000,0.0000,16.6881,3.2330,-10.3590,2.2735,-1.5728},
				{0.0200,30.9102,-1.8816,13.6102,2.9733,-9.6838,-2.1391,15.1800,3.0000,-10.0000,0.0000,16.7486,3.0267,-10.3162,2.1390,-1.5723},
				{0.0200,30.9101,-1.9376,13.6658,2.7788,-9.7240,-2.0101,15.2360,2.8000,-10.0000,0.0000,16.8050,2.8212,-10.2760,2.0101,-1.5718},
				{0.0200,30.9101,-1.9896,13.7175,2.5836,-9.7617,-1.8848,15.2880,2.6000,-10.0000,0.0000,16.8574,2.6164,-10.2383,1.8848,-1.5715},
				{0.0200,30.9100,-2.0376,13.7652,2.3876,-9.7969,-1.7615,15.3360,2.4000,-10.0000,0.0000,16.9056,2.4124,-10.2031,1.7615,-1.5713},
				{0.0200,30.9100,-2.0816,13.8091,2.1910,-9.8297,-1.6388,15.3800,2.2000,-10.0000,0.0000,16.9498,2.2090,-10.1703,1.6388,-1.5711},
				{0.0200,30.9100,-2.1216,13.8489,1.9938,-9.8600,-1.5155,15.4200,2.0000,-10.0000,0.0000,16.9899,2.0062,-10.1400,1.5155,-1.5710},
				{0.0200,30.9100,-2.1576,13.8849,1.7961,-9.8878,-1.3907,15.4560,1.8000,-10.0000,0.0000,17.0260,1.8039,-10.1122,1.3907,-1.5709},
				{0.0200,30.9100,-2.1896,13.9168,1.5978,-9.9131,-1.2637,15.4880,1.6000,-10.0000,0.0000,17.0580,1.6022,-10.0869,1.2637,-1.5708},
				{0.0200,30.9100,-2.2176,13.9448,1.3991,-9.9358,-1.1340,15.5160,1.4000,-10.0000,0.0000,17.0861,1.4009,-10.0642,1.1340,-1.5708},
				{0.0200,30.9100,-2.2300,13.9572,0.6181,-39.0511,-1455.7634,15.5284,1.2000,-10.0000,0.0000,17.0984,0.6183,-39.1312,-1453.3492,-1.5708},
				{0.0200,30.9100,-2.2300,13.9572,0.0000,-30.9042,407.3412,15.5284,1.0000,-10.0000,0.0000,17.0984,0.0000,-30.9135,410.8839,-1.5708},
				{0.0200,30.9100,-2.2300,13.9572,0.0000,0.0000,1545.2118,15.5284,0.8000,-10.0000,0.0000,17.0984,0.0000,0.0000,1545.6756,-1.5708},
				{0.0200,30.9100,-2.2300,13.9572,0.0000,0.0000,0.0000,15.5284,0.6000,-10.0000,0.0000,17.0984,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,13.9572,0.0000,0.0000,0.0000,15.5284,0.4000,-10.0000,0.0000,17.0984,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,13.9572,0.0000,0.0000,0.0000,15.5284,0.2000,-10.0000,0.0000,17.0984,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,13.9572,0.0000,0.0000,0.0000,15.5284,0.0000,-10.0000,0.0000,17.0984,0.0000,0.0000,0.0000,-1.5708},
				{0.0200,30.9100,-2.2300,13.9572,0.0000,0.0000,0.0000,15.5284,-0.2000,-10.0000,0.0000,17.0984,0.0000,0.0000,0.0000,-1.5708},

	    };

	@Override
	public double[][] getPath() {
	    return points;
	}
}