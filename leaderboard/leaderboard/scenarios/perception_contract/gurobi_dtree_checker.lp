\ Model collision_checker
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 npc_x_2 + 0 npc_y_2 + 0 yaw_0 + 0 yaw_1
Subject To
 R0: before_scenario + during_scenario + after_scenario = 1
 R1: - time + time_diff_0 = -43.4
 R2: - time + time_diff_1 = -43.4
 R3: - time + time_diff_2 = -43.4
 R4: - time + time_diff_3 = -43.4
 R5: 1.064162135124207 sin_yaw_0 + 2.450841665267944 cos_yaw_0 + rot_x_0
   = 0
 R6: 2.450841665267944 sin_yaw_0 - 1.064162135124207 cos_yaw_0 + rot_y_0
   = 0
 R7: - x_0 - rot_x_0 + trans_x_0 = 0
 R8: - y_0 - rot_y_0 + trans_y_0 = 0
 R9: - 1.064162135124207 sin_yaw_0 + 2.450841665267944 cos_yaw_0 + rot_x_0
   = 0
 R10: 2.450841665267944 sin_yaw_0 + 1.064162135124207 cos_yaw_0 + rot_y_0
   = 0
 R11: - x_0 - rot_x_0 + trans_x_0 = 0
 R12: - y_0 - rot_y_0 + trans_y_0 = 0
 R13: - 1.064162135124207 sin_yaw_0 - 2.450841665267944 cos_yaw_0 + rot_x_0
   = 0
 R14: - 2.450841665267944 sin_yaw_0 + 1.064162135124207 cos_yaw_0 + rot_y_0
   = 0
 R15: - x_0 - rot_x_0 + trans_x_0 = 0
 R16: - y_0 - rot_y_0 + trans_y_0 = 0
 R17: 1.064162135124207 sin_yaw_0 - 2.450841665267944 cos_yaw_0 + rot_x_0
   = 0
 R18: - 2.450841665267944 sin_yaw_0 - 1.064162135124207 cos_yaw_0 + rot_y_0
   = 0
 R19: - x_0 - rot_x_0 + trans_x_0 = 0
 R20: - y_0 - rot_y_0 + trans_y_0 = 0
 R21: - x + trans_x_0 + AM_x = 0
 R22: - y + trans_y_0 + AM_y = 0
 R23: trans_x_0 - trans_x_0 + AB_x = 0
 R24: trans_y_0 - trans_y_0 + AB_y = 0
 R25: trans_x_0 - trans_x_0 + AD_x = 0
 R26: trans_y_0 - trans_y_0 + AD_y = 0
 R27: 1.064162135124207 sin_yaw_1 + 2.450841665267944 cos_yaw_1 + rot_x_1
   = 0
 R28: 2.450841665267944 sin_yaw_1 - 1.064162135124207 cos_yaw_1 + rot_y_1
   = 0
 R29: - x_1 - rot_x_1 + trans_x_1 = 0
 R30: - y_1 - rot_y_1 + trans_y_1 = 0
 R31: - 1.064162135124207 sin_yaw_1 + 2.450841665267944 cos_yaw_1 + rot_x_1
   = 0
 R32: 2.450841665267944 sin_yaw_1 + 1.064162135124207 cos_yaw_1 + rot_y_1
   = 0
 R33: - x_1 - rot_x_1 + trans_x_1 = 0
 R34: - y_1 - rot_y_1 + trans_y_1 = 0
 R35: - 1.064162135124207 sin_yaw_1 - 2.450841665267944 cos_yaw_1 + rot_x_1
   = 0
 R36: - 2.450841665267944 sin_yaw_1 + 1.064162135124207 cos_yaw_1 + rot_y_1
   = 0
 R37: - x_1 - rot_x_1 + trans_x_1 = 0
 R38: - y_1 - rot_y_1 + trans_y_1 = 0
 R39: 1.064162135124207 sin_yaw_1 - 2.450841665267944 cos_yaw_1 + rot_x_1
   = 0
 R40: - 2.450841665267944 sin_yaw_1 - 1.064162135124207 cos_yaw_1 + rot_y_1
   = 0
 R41: - x_1 - rot_x_1 + trans_x_1 = 0
 R42: - y_1 - rot_y_1 + trans_y_1 = 0
 R43: - x + trans_x_1 + AM_x = 0
 R44: - y + trans_y_1 + AM_y = 0
 R45: trans_x_1 - trans_x_1 + AB_x = 0
 R46: trans_y_1 - trans_y_1 + AB_y = 0
 R47: trans_x_1 - trans_x_1 + AD_x = 0
 R48: trans_y_1 - trans_y_1 + AD_y = 0
 R49: - x + npc_x_0 + AM_x = 0
 R50: - y + npc_y_0 + AM_y = 0
 R51: npc_x_0 - npc_x_1 + AB_x = 0
 R52: npc_y_0 - npc_y_1 + AB_y = 0
 R53: npc_x_0 - npc_x_3 + AD_x = 0
 R54: npc_y_0 - npc_y_3 + AD_y = 0
 R55: - x + npc_x_0 + AM_x = 0
 R56: - y + npc_y_0 + AM_y = 0
 R57: npc_x_0 - npc_x_1 + AB_x = 0
 R58: npc_y_0 - npc_y_1 + AB_y = 0
 R59: npc_x_0 - npc_x_3 + AD_x = 0
 R60: npc_y_0 - npc_y_3 + AD_y = 0
 R61: frame_collision_0 + frame_collision_1 >= 1
 conjunct_2: time + y_0 + sin_yaw_0 - x_1 - y_1 - sin_yaw_1 - cos_yaw_1
   >= -113.167199999999
 R63: - time + time_diff >= -48.4
 R64: time + time_diff >= 48.4
 R65: - x_0 + var_diff_0_0 >= -154.4991445774219
 R66: x_0 + var_diff_0_0 >= 154.4991445774219
 R67: - y_0 + var_diff_0_1 >= -39.48931292676345
 R68: y_0 + var_diff_0_1 >= 39.48931292676345
 R69: - sin_yaw_0 + var_diff_0_2 >= -0.9682458365518543
 R70: sin_yaw_0 + var_diff_0_2 >= 0.9682458365518543
 R71: - cos_yaw_0 + var_diff_0_3 >= -0.25
 R72: cos_yaw_0 + var_diff_0_3 >= 0.25
 R73: - x_1 + var_diff_1_0 >= -153.920225739479
 R74: x_1 + var_diff_1_0 >= 153.920225739479
 R75: - y_1 + var_diff_1_1 >= -39.83317788736895
 R76: y_1 + var_diff_1_1 >= 39.83317788736895
 R77: - sin_yaw_1 + var_diff_1_2 >= -1
 R78: sin_yaw_1 + var_diff_1_2 >= 1
 R79: - cos_yaw_1 + var_diff_1_3 >= 0
 R80: cos_yaw_1 + var_diff_1_3 >= 0
 R81: time_different + var_different_0_0 + var_different_0_1
   + var_different_0_2 + var_different_0_3 + var_different_1_0
   + var_different_1_1 + var_different_1_2 + var_different_1_3 >= 1
 R82: - time + time_diff >= -43.4
 R83: time + time_diff >= 43.4
 R84: - x_0 + var_diff_0_0 >= -163.9200886487961
 R85: x_0 + var_diff_0_0 >= 163.9200886487961
 R86: - y_0 + var_diff_0_1 >= -39.84535765647887
 R87: y_0 + var_diff_0_1 >= 39.84535765647887
 R88: - sin_yaw_0 + var_diff_0_2 >= -1
 R89: sin_yaw_0 + var_diff_0_2 >= 1
 R90: - cos_yaw_0 + var_diff_0_3 >= 0
 R91: cos_yaw_0 + var_diff_0_3 >= 0
 R92: - x_1 + var_diff_1_0 >= -163.9200886487961
 R93: x_1 + var_diff_1_0 >= 163.9200886487961
 R94: - y_1 + var_diff_1_1 >= -39.84535765647887
 R95: y_1 + var_diff_1_1 >= 39.84535765647887
 R96: - sin_yaw_1 + var_diff_1_2 >= -1
 R97: sin_yaw_1 + var_diff_1_2 >= 1
 R98: - cos_yaw_1 + var_diff_1_3 >= 0
 R99: cos_yaw_1 + var_diff_1_3 >= 0
 R100: time_different + var_different_0_0 + var_different_0_1
   + var_different_0_2 + var_different_0_3 + var_different_1_0
   + var_different_1_1 + var_different_1_2 + var_different_1_3 >= 1
 R101: - time + time_diff >= -53.4
 R102: time + time_diff >= 53.4
 R103: - x_0 + var_diff_0_0 >= -139.0219283637065
 R104: x_0 + var_diff_0_0 >= 139.0219283637065
 R105: - y_0 + var_diff_0_1 >= -35.72495290841897
 R106: y_0 + var_diff_0_1 >= 35.72495290841897
 R107: - sin_yaw_0 + var_diff_0_2 >= -0.25
 R108: sin_yaw_0 + var_diff_0_2 >= 0.25
 R109: - cos_yaw_0 + var_diff_0_3 >= -0.9682458365518543
 R110: cos_yaw_0 + var_diff_0_3 >= 0.9682458365518543
 R111: - x_1 + var_diff_1_0 >= -139.9136458045379
 R112: x_1 + var_diff_1_0 >= 139.9136458045379
 R113: - y_1 + var_diff_1_1 >= -34.98849383355284
 R114: y_1 + var_diff_1_1 >= 34.98849383355284
 R115: - sin_yaw_1 + var_diff_1_2 >= -0.998044963916957
 R116: sin_yaw_1 + var_diff_1_2 >= 0.998044963916957
 R117: - cos_yaw_1 + var_diff_1_3 >= -0.0625
 R118: cos_yaw_1 + var_diff_1_3 >= 0.0625
 R119: time_different + var_different_0_0 + var_different_0_1
   + var_different_0_2 + var_different_0_3 + var_different_1_0
   + var_different_1_1 + var_different_1_2 + var_different_1_3 >= 1
 R120: - time + time_diff >= -48.4007327861163
 R121: time + time_diff >= 48.4007327861163
 R122: - x_0 + var_diff_0_0 >= -550989.7128927401
 R123: x_0 + var_diff_0_0 >= 550989.7128927401
 R124: - y_0 + var_diff_0_1 >= 545791.7178932844
 R125: y_0 + var_diff_0_1 >= -545791.7178932844
 R126: - sin_yaw_0 + var_diff_0_2 >= 0.9639256372140739
 R127: sin_yaw_0 + var_diff_0_2 >= -0.9639256372140739
 R128: - cos_yaw_0 + var_diff_0_3 >= -0.2660202998316723
 R129: cos_yaw_0 + var_diff_0_3 >= 0.2660202998316723
 R130: - x_1 + var_diff_1_0 >= -118.7054430839366
 R131: x_1 + var_diff_1_0 >= 118.7054430839366
 R132: - y_1 + var_diff_1_1 >= -37.50269935245267
 R133: y_1 + var_diff_1_1 >= 37.50269935245267
 R134: - sin_yaw_1 + var_diff_1_2 >= 0.00896753351268992
 R135: sin_yaw_1 + var_diff_1_2 >= -0.00896753351268992
 R136: - cos_yaw_1 + var_diff_1_3 >= 0.9999195054455708
 R137: cos_yaw_1 + var_diff_1_3 >= -0.9999195054455708
 R138: time_different + var_different_0_0 + var_different_0_1
   + var_different_0_2 + var_different_0_3 + var_different_1_0
   + var_different_1_1 + var_different_1_2 + var_different_1_3 >= 1
 R139: - time + time_diff >= -48.4007327861163
 R140: time + time_diff >= 48.4007327861163
 R141: - x_0 + var_diff_0_0 >= -550989.7128927401
 R142: x_0 + var_diff_0_0 >= 550989.7128927401
 R143: - y_0 + var_diff_0_1 >= 545791.7178932844
 R144: y_0 + var_diff_0_1 >= -545791.7178932844
 R145: - sin_yaw_0 + var_diff_0_2 >= 0.9639256372140739
 R146: sin_yaw_0 + var_diff_0_2 >= -0.9639256372140739
 R147: - cos_yaw_0 + var_diff_0_3 >= -0.2660202998316723
 R148: cos_yaw_0 + var_diff_0_3 >= 0.2660202998316723
 R149: - x_1 + var_diff_1_0 >= -118.7054430839366
 R150: x_1 + var_diff_1_0 >= 118.7054430839366
 R151: - y_1 + var_diff_1_1 >= -37.50269935245267
 R152: y_1 + var_diff_1_1 >= 37.50269935245267
 R153: - sin_yaw_1 + var_diff_1_2 >= 0.00896753351268992
 R154: sin_yaw_1 + var_diff_1_2 >= -0.00896753351268992
 R155: - cos_yaw_1 + var_diff_1_3 >= 0.9999195054455708
 R156: cos_yaw_1 + var_diff_1_3 >= -0.9999195054455708
 R157: time_different + var_different_0_0 + var_different_0_1
   + var_different_0_2 + var_different_0_3 + var_different_1_0
   + var_different_1_1 + var_different_1_2 + var_different_1_3 >= 1
 R158: - time + time_diff >= -48.4007327861163
 R159: time + time_diff >= 48.4007327861163
 R160: - x_0 + var_diff_0_0 >= -550989.7128927401
 R161: x_0 + var_diff_0_0 >= 550989.7128927401
 R162: - y_0 + var_diff_0_1 >= 545791.7178932844
 R163: y_0 + var_diff_0_1 >= -545791.7178932844
 R164: - sin_yaw_0 + var_diff_0_2 >= 0.9639256372140739
 R165: sin_yaw_0 + var_diff_0_2 >= -0.9639256372140739
 R166: - cos_yaw_0 + var_diff_0_3 >= -0.2660202998316723
 R167: cos_yaw_0 + var_diff_0_3 >= 0.2660202998316723
 R168: - x_1 + var_diff_1_0 >= -118.7054430839366
 R169: x_1 + var_diff_1_0 >= 118.7054430839366
 R170: - y_1 + var_diff_1_1 >= -37.50269935245267
 R171: y_1 + var_diff_1_1 >= 37.50269935245267
 R172: - sin_yaw_1 + var_diff_1_2 >= 0.00896753351268992
 R173: sin_yaw_1 + var_diff_1_2 >= -0.00896753351268992
 R174: - cos_yaw_1 + var_diff_1_3 >= 0.9999195054455708
 R175: cos_yaw_1 + var_diff_1_3 >= -0.9999195054455708
 R176: time_different + var_different_0_0 + var_different_0_1
   + var_different_0_2 + var_different_0_3 + var_different_1_0
   + var_different_1_1 + var_different_1_2 + var_different_1_3 >= 1
 R177: - time + time_diff >= -48.4007327861163
 R178: time + time_diff >= 48.4007327861163
 R179: - x_0 + var_diff_0_0 >= -550989.7128927401
 R180: x_0 + var_diff_0_0 >= 550989.7128927401
 R181: - y_0 + var_diff_0_1 >= 545791.7178932844
 R182: y_0 + var_diff_0_1 >= -545791.7178932844
 R183: - sin_yaw_0 + var_diff_0_2 >= 0.9639256372140739
 R184: sin_yaw_0 + var_diff_0_2 >= -0.9639256372140739
 R185: - cos_yaw_0 + var_diff_0_3 >= -0.2660202998316723
 R186: cos_yaw_0 + var_diff_0_3 >= 0.2660202998316723
 R187: - x_1 + var_diff_1_0 >= -118.7054430839366
 R188: x_1 + var_diff_1_0 >= 118.7054430839366
 R189: - y_1 + var_diff_1_1 >= -37.50269935245267
 R190: y_1 + var_diff_1_1 >= 37.50269935245267
 R191: - sin_yaw_1 + var_diff_1_2 >= 0.00896753351268992
 R192: sin_yaw_1 + var_diff_1_2 >= -0.00896753351268992
 R193: - cos_yaw_1 + var_diff_1_3 >= 0.9999195054455708
 R194: cos_yaw_1 + var_diff_1_3 >= -0.9999195054455708
 R195: time_different + var_different_0_0 + var_different_0_1
   + var_different_0_2 + var_different_0_3 + var_different_1_0
   + var_different_1_1 + var_different_1_2 + var_different_1_3 >= 1
 R196: - time + time_diff >= -48.4007327861163
 R197: time + time_diff >= 48.4007327861163
 R198: - x_0 + var_diff_0_0 >= -550989.7128927401
 R199: x_0 + var_diff_0_0 >= 550989.7128927401
 R200: - y_0 + var_diff_0_1 >= 545791.7178932844
 R201: y_0 + var_diff_0_1 >= -545791.7178932844
 R202: - sin_yaw_0 + var_diff_0_2 >= 0.9639256372140739
 R203: sin_yaw_0 + var_diff_0_2 >= -0.9639256372140739
 R204: - cos_yaw_0 + var_diff_0_3 >= -0.2660202998316723
 R205: cos_yaw_0 + var_diff_0_3 >= 0.2660202998316723
 R206: - x_1 + var_diff_1_0 >= -118.7054430839366
 R207: x_1 + var_diff_1_0 >= 118.7054430839366
 R208: - y_1 + var_diff_1_1 >= -37.50269935245267
 R209: y_1 + var_diff_1_1 >= 37.50269935245267
 R210: - sin_yaw_1 + var_diff_1_2 >= 0.00896753351268992
 R211: sin_yaw_1 + var_diff_1_2 >= -0.00896753351268992
 R212: - cos_yaw_1 + var_diff_1_3 >= 0.9999195054455708
 R213: cos_yaw_1 + var_diff_1_3 >= -0.9999195054455708
 R214: time_different + var_different_0_0 + var_different_0_1
   + var_different_0_2 + var_different_0_3 + var_different_1_0
   + var_different_1_1 + var_different_1_2 + var_different_1_3 >= 1
 qc0: disp_x_0 + [ 0.9999862909317017 s * time_diff_0 ] = 0
 qc1: disp_y_0 + [ 0.00121797691099346 s * time_diff_0 ] = 0
 qc2: disp_x_1 + [ 0.9999862909317017 s * time_diff_1 ] = 0
 qc3: disp_y_1 + [ 0.00121797691099346 s * time_diff_1 ] = 0
 qc4: disp_x_2 + [ 0.9999862909317017 s * time_diff_2 ] = 0
 qc5: disp_y_2 + [ 0.00121797691099346 s * time_diff_2 ] = 0
 qc6: disp_x_3 + [ 0.9999862909317017 s * time_diff_3 ] = 0
 qc7: disp_y_3 + [ 0.00121797691099346 s * time_diff_3 ] = 0
 qc8: AM_AB + [ - AM_x * AB_x - AM_y * AB_y ] = 0
 qc9: AM_AD + [ - AM_x * AD_x - AM_y * AD_y ] = 0
 qc10: AB_AB + [ - AB_x ^2 - AB_y ^2 ] = 0
 qc11: AD_AD + [ - AD_x ^2 - AD_y ^2 ] = 0
 qc12: ego_collision_0 + [ - inside_AB * inside_AD ] = 0
 qc13: AM_AB + [ - AM_x * AB_x - AM_y * AB_y ] = 0
 qc14: AM_AD + [ - AM_x * AD_x - AM_y * AD_y ] = 0
 qc15: AB_AB + [ - AB_x ^2 - AB_y ^2 ] = 0
 qc16: AD_AD + [ - AD_x ^2 - AD_y ^2 ] = 0
 qc17: ego_collision_1 + [ - inside_AB * inside_AD ] = 0
 qc18: AM_AB + [ - AM_x * AB_x - AM_y * AB_y ] = 0
 qc19: AM_AD + [ - AM_x * AD_x - AM_y * AD_y ] = 0
 qc20: AB_AB + [ - AB_x ^2 - AB_y ^2 ] = 0
 qc21: AD_AD + [ - AD_x ^2 - AD_y ^2 ] = 0
 qc22: npc_collision_0 + [ - inside_AB * inside_AD ] = 0
 qc23: AM_AB + [ - AM_x * AB_x - AM_y * AB_y ] = 0
 qc24: AM_AD + [ - AM_x * AD_x - AM_y * AD_y ] = 0
 qc25: AB_AB + [ - AB_x ^2 - AB_y ^2 ] = 0
 qc26: AD_AD + [ - AD_x ^2 - AD_y ^2 ] = 0
 qc27: npc_collision_1 + [ - inside_AB * inside_AD ] = 0
 qc28: frame_collision_0 + [ - ego_collision_0 * npc_collision_0 ] = 0
 qc29: frame_collision_1 + [ - ego_collision_1 * npc_collision_1 ] = 0
 GC0: before_scenario = 1 -> time <= 43.4
 GC1: during_scenario = 1 -> time >= 43.4
 GC2: during_scenario = 1 -> time <= 53.4
 GC3: after_scenario = 1 -> time >= 53.4
 GC4: before_scenario = 1 -> s = 0
 GC5: after_scenario = 1 -> s = 0
 GC6: during_scenario = 1 -> s >= 2
 GC7: during_scenario = 1 -> s <= 15
 GC8: before_scenario = 1 -> npc_x_0 = 162.8559265136719
 GC9: before_scenario = 1 -> npc_y_0 = 37.39451599121094
 GC10: during_scenario = 1 -> npc_x_0 - disp_x_0 = 162.8559265136719
 GC11: during_scenario = 1 -> npc_y_0 - disp_y_0 = 37.39451599121094
 GC12: after_scenario = 1 -> disp_x_0 = 0
 GC13: after_scenario = 1 -> disp_y_0 = 0
 GC14: before_scenario = 1 -> npc_x_1 = 162.8563842773438
 GC15: before_scenario = 1 -> npc_y_1 = 37.02199935913086
 GC16: during_scenario = 1 -> npc_x_1 - disp_x_1 = 162.8563842773438
 GC17: during_scenario = 1 -> npc_y_1 - disp_y_1 = 37.02199935913086
 GC18: after_scenario = 1 -> disp_x_1 = 0
 GC19: after_scenario = 1 -> disp_y_1 = 0
 GC20: before_scenario = 1 -> npc_x_2 = 161.2130889892578
 GC21: before_scenario = 1 -> npc_y_2 = 37.39252090454102
 GC22: during_scenario = 1 -> npc_x_2 - disp_x_2 = 161.2130889892578
 GC23: during_scenario = 1 -> npc_y_2 - disp_y_2 = 37.39252090454102
 GC24: after_scenario = 1 -> disp_x_2 = 0
 GC25: after_scenario = 1 -> disp_y_2 = 0
 GC26: before_scenario = 1 -> npc_x_3 = 161.2135467529297
 GC27: before_scenario = 1 -> npc_y_3 = 37.02000427246094
 GC28: during_scenario = 1 -> npc_x_3 - disp_x_3 = 161.2135467529297
 GC29: during_scenario = 1 -> npc_y_3 - disp_y_3 = 37.02000427246094
 GC30: after_scenario = 1 -> disp_x_3 = 0
 GC31: after_scenario = 1 -> disp_y_3 = 0
 GC34: inside_AB = 1 -> AM_AB >= 0
 GC35: inside_AB = 1 -> AM_AB - AB_AB <= 0
 GC36: inside_AD = 1 -> AM_AD >= 0
 GC37: inside_AD = 1 -> AM_AD - AD_AD <= 0
 GC40: inside_AB = 1 -> AM_AB >= 0
 GC41: inside_AB = 1 -> AM_AB - AB_AB <= 0
 GC42: inside_AD = 1 -> AM_AD >= 0
 GC43: inside_AD = 1 -> AM_AD - AD_AD <= 0
 GC44: inside_AB = 1 -> AM_AB >= 0
 GC45: inside_AB = 1 -> AM_AB - AB_AB <= 0
 GC46: inside_AD = 1 -> AM_AD >= 0
 GC47: inside_AD = 1 -> AM_AD - AD_AD <= 0
 GC48: inside_AB = 1 -> AM_AB >= 0
 GC49: inside_AB = 1 -> AM_AB - AB_AB <= 0
 GC50: inside_AD = 1 -> AM_AD >= 0
 GC51: inside_AD = 1 -> AM_AD - AD_AD <= 0
 GC52: time_different = 1 -> time_diff >= 0.001
 GC53: var_different_0_0 = 1 -> var_diff_0_0 >= 0.001
 GC54: var_different_0_1 = 1 -> var_diff_0_1 >= 0.001
 GC55: var_different_0_2 = 1 -> var_diff_0_2 >= 0.001
 GC56: var_different_0_3 = 1 -> var_diff_0_3 >= 0.001
 GC57: var_different_1_0 = 1 -> var_diff_1_0 >= 0.001
 GC58: var_different_1_1 = 1 -> var_diff_1_1 >= 0.001
 GC59: var_different_1_2 = 1 -> var_diff_1_2 >= 0.001
 GC60: var_different_1_3 = 1 -> var_diff_1_3 >= 0.001
 GC61: time_different = 1 -> time_diff >= 0.001
 GC62: var_different_0_0 = 1 -> var_diff_0_0 >= 0.001
 GC63: var_different_0_1 = 1 -> var_diff_0_1 >= 0.001
 GC64: var_different_0_2 = 1 -> var_diff_0_2 >= 0.001
 GC65: var_different_0_3 = 1 -> var_diff_0_3 >= 0.001
 GC66: var_different_1_0 = 1 -> var_diff_1_0 >= 0.001
 GC67: var_different_1_1 = 1 -> var_diff_1_1 >= 0.001
 GC68: var_different_1_2 = 1 -> var_diff_1_2 >= 0.001
 GC69: var_different_1_3 = 1 -> var_diff_1_3 >= 0.001
 GC70: time_different = 1 -> time_diff >= 0.001
 GC71: var_different_0_0 = 1 -> var_diff_0_0 >= 0.001
 GC72: var_different_0_1 = 1 -> var_diff_0_1 >= 0.001
 GC73: var_different_0_2 = 1 -> var_diff_0_2 >= 0.001
 GC74: var_different_0_3 = 1 -> var_diff_0_3 >= 0.001
 GC75: var_different_1_0 = 1 -> var_diff_1_0 >= 0.001
 GC76: var_different_1_1 = 1 -> var_diff_1_1 >= 0.001
 GC77: var_different_1_2 = 1 -> var_diff_1_2 >= 0.001
 GC78: var_different_1_3 = 1 -> var_diff_1_3 >= 0.001
 GC79: time_different = 1 -> time_diff >= 0.001
 GC80: var_different_0_0 = 1 -> var_diff_0_0 >= 0.001
 GC81: var_different_0_1 = 1 -> var_diff_0_1 >= 0.001
 GC82: var_different_0_2 = 1 -> var_diff_0_2 >= 0.001
 GC83: var_different_0_3 = 1 -> var_diff_0_3 >= 0.001
 GC84: var_different_1_0 = 1 -> var_diff_1_0 >= 0.001
 GC85: var_different_1_1 = 1 -> var_diff_1_1 >= 0.001
 GC86: var_different_1_2 = 1 -> var_diff_1_2 >= 0.001
 GC87: var_different_1_3 = 1 -> var_diff_1_3 >= 0.001
 GC88: time_different = 1 -> time_diff >= 0.001
 GC89: var_different_0_0 = 1 -> var_diff_0_0 >= 0.001
 GC90: var_different_0_1 = 1 -> var_diff_0_1 >= 0.001
 GC91: var_different_0_2 = 1 -> var_diff_0_2 >= 0.001
 GC92: var_different_0_3 = 1 -> var_diff_0_3 >= 0.001
 GC93: var_different_1_0 = 1 -> var_diff_1_0 >= 0.001
 GC94: var_different_1_1 = 1 -> var_diff_1_1 >= 0.001
 GC95: var_different_1_2 = 1 -> var_diff_1_2 >= 0.001
 GC96: var_different_1_3 = 1 -> var_diff_1_3 >= 0.001
 GC97: time_different = 1 -> time_diff >= 0.001
 GC98: var_different_0_0 = 1 -> var_diff_0_0 >= 0.001
 GC99: var_different_0_1 = 1 -> var_diff_0_1 >= 0.001
 GC100: var_different_0_2 = 1 -> var_diff_0_2 >= 0.001
 GC101: var_different_0_3 = 1 -> var_diff_0_3 >= 0.001
 GC102: var_different_1_0 = 1 -> var_diff_1_0 >= 0.001
 GC103: var_different_1_1 = 1 -> var_diff_1_1 >= 0.001
 GC104: var_different_1_2 = 1 -> var_diff_1_2 >= 0.001
 GC105: var_different_1_3 = 1 -> var_diff_1_3 >= 0.001
 GC106: time_different = 1 -> time_diff >= 0.001
 GC107: var_different_0_0 = 1 -> var_diff_0_0 >= 0.001
 GC108: var_different_0_1 = 1 -> var_diff_0_1 >= 0.001
 GC109: var_different_0_2 = 1 -> var_diff_0_2 >= 0.001
 GC110: var_different_0_3 = 1 -> var_diff_0_3 >= 0.001
 GC111: var_different_1_0 = 1 -> var_diff_1_0 >= 0.001
 GC112: var_different_1_1 = 1 -> var_diff_1_1 >= 0.001
 GC113: var_different_1_2 = 1 -> var_diff_1_2 >= 0.001
 GC114: var_different_1_3 = 1 -> var_diff_1_3 >= 0.001
 GC115: time_different = 1 -> time_diff >= 0.001
 GC116: var_different_0_0 = 1 -> var_diff_0_0 >= 0.001
 GC117: var_different_0_1 = 1 -> var_diff_0_1 >= 0.001
 GC118: var_different_0_2 = 1 -> var_diff_0_2 >= 0.001
 GC119: var_different_0_3 = 1 -> var_diff_0_3 >= 0.001
 GC120: var_different_1_0 = 1 -> var_diff_1_0 >= 0.001
 GC121: var_different_1_1 = 1 -> var_diff_1_1 >= 0.001
 GC122: var_different_1_2 = 1 -> var_diff_1_2 >= 0.001
 GC123: var_different_1_3 = 1 -> var_diff_1_3 >= 0.001
Bounds
 43.4 <= time <= 53.4
 2 <= s <= 15
 x free
 y free
 x_0 free
 y_0 free
 -1 <= sin_yaw_0 <= 1
 -1 <= cos_yaw_0 <= 1
 x_1 free
 y_1 free
 -1 <= sin_yaw_1 <= 1
 -1 <= cos_yaw_1 <= 1
 npc_x_0 free
 npc_y_0 free
 npc_x_1 free
 npc_y_1 free
 npc_x_2 free
 npc_y_2 free
 npc_x_3 free
 npc_y_3 free
 time_diff_0 free
 disp_x_0 free
 disp_y_0 free
 time_diff_1 free
 disp_x_1 free
 disp_y_1 free
 time_diff_2 free
 disp_x_2 free
 disp_y_2 free
 time_diff_3 free
 disp_x_3 free
 disp_y_3 free
 -3.141592653589793 <= yaw_0 <= 3.141592653589793
 rot_x_0 free
 rot_y_0 free
 trans_x_0 free
 trans_y_0 free
 rot_x_0 free
 rot_y_0 free
 trans_x_0 free
 trans_y_0 free
 rot_x_0 free
 rot_y_0 free
 trans_x_0 free
 trans_y_0 free
 rot_x_0 free
 rot_y_0 free
 trans_x_0 free
 trans_y_0 free
 AM_x free
 AM_y free
 AB_x free
 AB_y free
 AD_x free
 AD_y free
 AM_AB free
 AM_AD free
 -3.141592653589793 <= yaw_1 <= 3.141592653589793
 rot_x_1 free
 rot_y_1 free
 trans_x_1 free
 trans_y_1 free
 rot_x_1 free
 rot_y_1 free
 trans_x_1 free
 trans_y_1 free
 rot_x_1 free
 rot_y_1 free
 trans_x_1 free
 trans_y_1 free
 rot_x_1 free
 rot_y_1 free
 trans_x_1 free
 trans_y_1 free
 AM_x free
 AM_y free
 AB_x free
 AB_y free
 AD_x free
 AD_y free
 AM_AB free
 AM_AD free
 AM_x free
 AM_y free
 AB_x free
 AB_y free
 AD_x free
 AD_y free
 AM_AB free
 AM_AD free
 AM_x free
 AM_y free
 AB_x free
 AB_y free
 AD_x free
 AD_y free
 AM_AB free
 AM_AD free
Binaries
 ego_collision_0 npc_collision_0 ego_collision_1 npc_collision_1
 before_scenario during_scenario after_scenario inside_AB inside_AD
 inside_AB inside_AD inside_AB inside_AD inside_AB inside_AD
 frame_collision_0 frame_collision_1 time_different var_different_0_0
 var_different_0_1 var_different_0_2 var_different_0_3 var_different_1_0
 var_different_1_1 var_different_1_2 var_different_1_3 time_different
 var_different_0_0 var_different_0_1 var_different_0_2 var_different_0_3
 var_different_1_0 var_different_1_1 var_different_1_2 var_different_1_3
 time_different var_different_0_0 var_different_0_1 var_different_0_2
 var_different_0_3 var_different_1_0 var_different_1_1 var_different_1_2
 var_different_1_3 time_different var_different_0_0 var_different_0_1
 var_different_0_2 var_different_0_3 var_different_1_0 var_different_1_1
 var_different_1_2 var_different_1_3 time_different var_different_0_0
 var_different_0_1 var_different_0_2 var_different_0_3 var_different_1_0
 var_different_1_1 var_different_1_2 var_different_1_3 time_different
 var_different_0_0 var_different_0_1 var_different_0_2 var_different_0_3
 var_different_1_0 var_different_1_1 var_different_1_2 var_different_1_3
 time_different var_different_0_0 var_different_0_1 var_different_0_2
 var_different_0_3 var_different_1_0 var_different_1_1 var_different_1_2
 var_different_1_3 time_different var_different_0_0 var_different_0_1
 var_different_0_2 var_different_0_3 var_different_1_0 var_different_1_1
 var_different_1_2 var_different_1_3
General Constraints
 GC32: sin_yaw_0 = SIN ( yaw_0 )
 GC33: cos_yaw_0 = COS ( yaw_0 )
 GC38: sin_yaw_1 = SIN ( yaw_1 )
 GC39: cos_yaw_1 = COS ( yaw_1 )
End
