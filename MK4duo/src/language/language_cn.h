/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Chinese
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_CN_H
#define LANGUAGE_CN_H

#define MAPPER_NON                  // For direct asci codes
#define DISPLAY_CHARSET_ISO10646_1  // use the better font on full graphic displays.


#define WELCOME_MSG                          MACHINE_NAME "准备好了。"
#define MSG_SD                               " SD "
#define MSG_SD_INSERTED                      MSG_SD "插入"
#define MSG_SD_REMOVED                       MSG_SD "删除"
#define MSG_MAIN                             "主页"
#define MSG_AUTOSTART                        "自动启动"
#define MSG_DISABLE_STEPPERS                 "禁用电机"
#define MSG_AUTO_HOME                        "自动回位"
#define MSG_AUTO_HOME_X                      "x回位 "
#define MSG_AUTO_HOME_Y                      "y回位"
#define MSG_AUTO_HOME_Z                      "z回位"
#define MSG_LEVEL_BED_HOMING                 "XYZ归零"
#define MSG_LEVEL_BED_WAITING                "点击开始"
#define MSG_LEVEL_BED_NEXT_POINT             "下一步"
#define MSG_LEVEL_BED_DONE                   "找平完成！"
#define MSG_LEVEL_BED_CANCEL                 "取消"
#define MSG_SET_HOME_OFFSETS                 "起点偏移"
#define MSG_SET_ORIGIN                       "设置起点"
#define MSG_PREHEAT                          "预热"
#define MSG_PREHEAT_1                        MSG_PREHEAT "PAL"
#define MSG_PREHEAT_1_N                      MSG_PREHEAT_1 "  "
#define MSG_PREHEAT_1_ALL                    MSG_PREHEAT_1 "所有"
#define MSG_PREHEAT_1_BEDONLY                MSG_PREHEAT_1 "热床"
#define MSG_PREHEAT_1_SETTINGS               MSG_PREHEAT_1 "conf "
#define MSG_PREHEAT_2                        MSG_PREHEAT " ABS "
#define MSG_PREHEAT_2_N                      MSG_PREHEAT_2 "  "
#define MSG_PREHEAT_2_ALL                    MSG_PREHEAT_2 "所有"
#define MSG_PREHEAT_2_BEDONLY                MSG_PREHEAT_2 "热床"
#define MSG_PREHEAT_2_SETTINGS               MSG_PREHEAT_2 "conf "
#define MSG_PREHEAT_3                        MSG_PREHEAT " GUM "
#define MSG_PREHEAT_3_N                      MSG_PREHEAT_3 "  "
#define MSG_PREHEAT_3_ALL                    MSG_PREHEAT_3 "所有"
#define MSG_PREHEAT_3_BEDONLY                MSG_PREHEAT_3 "热床"
#define MSG_PREHEAT_3_SETTINGS               MSG_PREHEAT_3 "conf "
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE      " 喷头太冷改变长丝"
#define MSG_COOLDOWN                         "冷却"
#define MSG_SWITCH_PS_ON                     "开电源"
#define MSG_SWITCH_PS_OFF                    "关电源"
#define MSG_EXTRUDE                          "挤出"
#define MSG_RETRACT                          "收回"
#define MSG_PURGE                            "清除"
#define MSG_LEVEL_BED                        "平床"
#define MSG_SPEED                            "速度"
#define MSG_NOZZLE                           "喷嘴"
#define MSG_BED                              "床"
#define MSG_CHAMBER                          "机室"
#define MSG_COOLER                           "冷却器"
#define MSG_BED_Z                            "床 Z"
#define MSG_FAN_SPEED                        "风扇转速"
#define MSG_FLOW                             "流量"
#define msg_control域                         "控制"
#define MSG_FIX_LOSE_STEPS                   "修改轴步"
#define MSG_MIN                              LCD_STR_THERMOMETER "最小"
#define MSG_MAX                              LCD_STR_THERMOMETER "最大"
#define MSG_FACTOR                           LCD_STR_THERMOMETER "实际"
#define MSG_IDLEOOZING                       "渗出"
#define MSG_AUTOTEMP                         "自动恒温"
#define MSG_ON                               "开"
#define MSG_OFF                              "关"
#define MSG_PID_P                            " PID-P "
#define MSG_PID_I                            " PID-I "
#define MSG_PID_D                            " PID-D "
#define MSG_PID_C                            " PID-C "
#define MSG_SELECT                           "选择"
#define MSG_H1                               " H1 "
#define MSG_H2                               " H2 "
#define MSG_H3                               " H3 "
#define MSG_ACC                              "加速度"
#define MSG_VXY_JERK                         " Vxy-jerk"
#define MSG_VZ_JERK                          " VZ-jerk"
#define MSG_VE_JERK                          " VE-jerk"
#define MSG_VMAX                             "V最大"
#define MSG_X                                "x"
#define MSG_Y                                "y"
#define MSG_Z                                "z"
#define MSG_E                                "e"
#define MSG_MOVE                             "移动"
#define MSG_MOVE_AXIS                        MSG_MOVE "轴"
#define MSG_MOVE_X                           MSG_MOVE "  " MSG_X
#define MSG_MOVE_Y                           MSG_MOVE "  " MSG_Y
#define MSG_MOVE_Z                           MSG_MOVE "  " MSG_Z
#define MSG_MOVE_01MM                        MSG_MOVE " 0.1毫米"
#define MSG_MOVE_1MM                         MSG_MOVE " 1毫米"
#define MSG_MOVE_10MM                        MSG_MOVE " 10毫米"
#define MSG_MOVE_E                           "挤出机"
#define MSG_VMIN                             "最小"
#define MSG_VTRAV_MIN                        " VTrav分钟"
#define MSG_AMAX                             "A最大"
#define MSG_A_RETRACT                        " A-收回"
#define MSG_A_TRAVEL                         " A-移动"
#define MSG_XSTEPS                           MSG_X "步/毫米"
#define MSG_YSTEPS                           MSG_Y "步/毫米"
#define MSG_ZSTEPS                           MSG_Z "步/毫米"
#define MSG_E0STEPS                          MSG_E " 0步/毫米"
#define MSG_E1STEPS                          MSG_E " 1步/毫米"
#define MSG_E2STEPS                          MSG_E " 2步/毫米"
#define MSG_E3STEPS                          MSG_E " 3步/毫米"
#define MSG_TEMPERATURE                      "温度"
#define MSG_MOTION                           "运动"
#define MSG_FILAMENT                         "灯丝"
#define MSG_VOLUMETRIC_ENABLED               MSG_E "在MM3 "
#define MSG_FILAMENT_SIZE_EXTRUDER           "耗材尺寸"
#define MSG_CONTRAST                         " LCD对比度"
#define MSG_STORE_EPROM                      "存储内存"
#define MSG_LOAD_EPROM                       "装载存储器"
#define MSG_RESTORE_FAILSAFE                 "还原"
#define MSG_REFRESH                          "刷新"
#define MSG_WATCH                            "信息界面"
#define MSG_PREPARE                          "准备"
#define MSG_TUNE                             "调整"
#define MSG_PAUSE_PRINT                      "暂停打印"
#define MSG_RESUME_PRINT                     "继续打印"
#define MSG_STOP_PRINT                       "停止印刷"
#define MSG_STOP_SAVE_PRINT                  "停止并保存"
#define MSG_CARD_MENU                        "sd卡打印"
#define MSG_NO_CARD                          "不从sd"
#define MSG_DWELL                            "睡眠...... "
#define MSG_USERWAIT                         "等待用户...... "
#define MSG_RESUMING                         "恢复打印"
#define MSG_PRINT_ABORTED                    "打印中止"
#define MSG_NO_MOVE                          "不移动。"
#define MSG_KILLED                           "断开。"
#define MSG_STOPPED                          "已停止。"
#define MSG_CONTROL_RETRACT                  "控制收回"
#define MSG_CONTROL_RETRACT_SWAP             "收回毫米"
#define MSG_CONTROL_RETRACTF                 "收回速度"
#define MSG_CONTROL_RETRACT_ZLIFT            "多少毫米"
#define MSG_CONTROL_RETRACT_RECOVER          "返回+毫米"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP     "收回返回+毫米"
#define MSG_CONTROL_RETRACT_RECOVERF         "收返速度"
#define MSG_AUTORETRACT                      "自动缩回"
#define MSG_FILAMENT_CHANGE                  "更改长丝"
#define MSG_INIT_SDCARD                      "初始化sd"
#define MSG_CNG_SDCARD                       "更改sd"
#define MSG_ZPROBE_OUT                       "z探测.床"
#define MSG_BLTOUCH_SELFTEST                 " 触摸探测"
#define MSG_RESET_BLTOUCH                    "复位触摸 "
#define MSG_HOME                             "归位"
#define MSG_FIRST                            "首次"
#define MSG_ZPROBE_ZOFFSET                   "探针偏移"
#define MSG_BABYSTEP                         " 初步 "
#define MSG_BABYSTEP_X                       MSG_BABYSTEP "  " MSG_X
#define MSG_BABYSTEP_Y                       MSG_BABYSTEP "  " MSG_Y
#define MSG_BABYSTEP_Z                       MSG_BABYSTEP "  " MSG_Z
#define MSG_ENDSTOP_ABORT                    "挡块放弃"
#define MSG_HEATING_FAILED_LCD               "加热失败"
#define MSG_ERR_REDUNDANT_TEMP               "温度误差"
#define MSG_THERMAL_RUNAWAY                  "热失控"
#define MSG_AD595                            "AD595偏移和增益"
#define MSG_ERR_MAXTEMP                      "最大温度误差"
#define MSG_ERR_MINTEMP                      "最小温度误差"
#define MSG_ERR_MAXTEMP_BED                  "最大温度床错误"
#define MSG_ERR_MINTEMP_BED                  "最小温度床错误"
#define MSG_ERR_Z_HOMING                     "G28?错误"
#define MSG_ERR_MAXTEMP_CHAMBER              "最大温度室误差"
#define MSG_ERR_MINTEMP_CHAMBER              "最小温度室误差"
#define MSG_ERR_MAXTEMP_COOLER               "最大温度冷却器错误"
#define MSG_ERR_MINTEMP_COOLER               "最小温度冷却器错误"
#define MSG_HALTED                           "打印机停止"
#define MSG_PLEASE_RESET                     "请重置"
#define MSG_END_DAY                          "天"
#define MSG_END_HOUR                         "小时"
#define MSG_END_MINUTE                       "分钟"
#define MSG_PRINT_TIME                       "打印时间"

#define MSG_ENDSTOPS_HIT                     "终点挡块击中："
#define MSG_BABYSTEPPING                     " Babystepping "
#define MSG_BABYSTEPPING_X                   MSG_BABYSTEPPING "  " MSG_X
#define MSG_BABYSTEPPING_Y                   MSG_BABYSTEPPING "  " MSG_Y
#define MSG_BABYSTEPPING_Z                   MSG_BABYSTEPPING "  " MSG_Z

#define MSG_ENDSTOP_XS                       MSG_X
#define MSG_ENDSTOP_YS                       MSG_Y
#define MSG_ENDSTOP_ZS                       MSG_Z
#define MSG_ENDSTOP_ZPS                      MSG_Z " P "
#define MSG_ENDSTOP_ES                       MSG_E

// Calibrate Delta
#define MSG_DELTA_CALIBRATE                  "三角洲校准"
#define MSG_DELTA_CALIBRATE_X                "校准" MSG_X
#define MSG_DELTA_CALIBRATE_Y                "校准" MSG_Y
#define MSG_DELTA_CALIBRATE_Z                "校准" MSG_Z
#define MSG_DELTA_CALIBRATE_CENTER           "校准中心"

// Info printers
#define MSG_INFO_MENU                        "关于打印机"
#define MSG_INFO_FIRMWARE_MENU               "固件信息"
#define MSG_INFO_STATS_MENU                  "打印状态"
#define MSG_INFO_BOARD_MENU                  "信息栏"
#define MSG_INFO_THERMISTOR_MENU             "热敏电阻"
#define MSG_INFO_EXTRUDERS                   "挤出机"
#define MSG_INFO_HOTENDS                     "喷头"
#define MSG_INFO_BED                         "热床"
#define MSG_INFO_CHAMBER                     "热室"
#define MSG_INFO_COOLER                      "冷却器"
#define MSG_INFO_BAUDRATE                    "波特"
#define MSG_INFO_PROTOCOL                    "协议"
#define MSG_INFO_TOTAL_PRINTS                "打印"
#define MSG_INFO_FINISHED_PRINTS             "打印好"
#define MSG_INFO_ON_TIME                     "开机时间"
#define MSG_INFO_PRINT_TIME                  "打印时间"
#define MSG_INFO_FILAMENT_USAGE              "文件"
#define MSG_INFO_PWRCONSUMED                 " PWR "
#define MSG_INFO_MIN_TEMP                    "最低温度"
#define MSG_INFO_MAX_TEMP                    "最高温度"
#define MSG_INFO_PSU                         "电源"

// FILAMENT_CHANGE_FEATURE
#define MSG_FILAMENT_CHANGE_HEADER           "改变材料"
#define MSG_FILAMENT_CHANGE_INIT_1           "等待开始"
#define MSG_FILAMENT_CHANGE_INIT_2           "材料"
#define MSG_FILAMENT_CHANGE_INIT_3           "改变"
#define MSG_FILAMENT_CHANGE_UNLOAD_1         "等待"
#define MSG_FILAMENT_CHANGE_UNLOAD_2         "材料卸载"
#define MSG_FILAMENT_CHANGE_UNLOAD_3         " "
#define MSG_FILAMENT_CHANGE_INSERT_1         "插入材料"
#define MSG_FILAMENT_CHANGE_INSERT_2         "然后按下按钮"
#define MSG_FILAMENT_CHANGE_INSERT_3         "继续...... "
#define MSG_FILAMENT_CHANGE_LOAD_1           "等待"
#define MSG_FILAMENT_CHANGE_LOAD_2           "材料负荷"
#define MSG_FILAMENT_CHANGE_LOAD_3           " "
#define MSG_FILAMENT_CHANGE_EXTRUDE_1        "等待"
#define MSG_FILAMENT_CHANGE_EXTRUDE_2        "材料挤出"
#define MSG_FILAMENT_CHANGE_EXTRUDE_3        " "
#define MSG_FILAMENT_CHANGE_OPTION_HEADER    "下一步是什么？"
#define MSG_FILAMENT_CHANGE_OPTION_EXTRUDE   "挤出更多的"
#define MSG_FILAMENT_CHANGE_OPTION_RESUME    "继续打印"
#define MSG_FILAMENT_CHANGE_RESUME_1         "等待打印"
#define MSG_FILAMENT_CHANGE_RESUME_2         "恢复"
#define MSG_FILAMENT_CHANGE_RESUME_3         " "

// Heater
#define MSG_HEATING                          "加热...... "
#define MSG_HEATING_COMPLETE                 "加热完成"
#define MSG_BED_HEATING                      "热床"
#define MSG_BED_DONE                         "床完成"
#define MSG_CHAMBER_HEATING                  "室内加热"
#define MSG_CHAMBER_DONE                     "室内完成"
#define MSG_COOLER_COOLING                   "冷...... "
#define MSG_COOLER_DONE                      "冷却完成"

// Extra
#define MSG_LASER                            "激光预置"
#define MSG_CONFIG                           "配置"
#define MSG_E_BOWDEN_LENGTH                  MSG_EXTRUDE "  "字符串化（BOWDEN_LENGTH） "毫米"
#define MSG_R_BOWDEN_LENGTH                  MSG_RETRACT "  "字符串化（BOWDEN_LENGTH） "毫米"
#define MSG_PURGE_XMM                        MSG_PURGE "  "字符串化（LCD_PURGE_LENGTH） "毫米"
#define MSG_RETRACT_XMM                      MSG_RETRACT "  "字符串化（LCD_RETRACT_LENGTH） "毫米"
#define MSG_SAVED_POS                        "保存位置"
#define MSG_RESTORING_POS                    "还原位置"
#define MSG_INVALID_POS_SLOT                 "无效的插槽，插槽总额："

// Rfid module
#define MSG_RFID_SPOOL                       "第E后台"
#define MSG_RFID_BRAND                       "品牌："
#define MSG_RFID_TYPE                        "类型："
#define MSG_RFID_COLOR                       "颜色"
#define MSG_RFID_SIZE                        "尺寸："
#define MSG_RFID_TEMP_HOTEND                 "喷头温度"
#define MSG_RFID_TEMP_BED                    "热床："
#define MSG_RFID_TEMP_USER_HOTEND            "用户喷头温度"
#define MSG_RFID_TEMP_USER_BED               "用户热床温度"
#define MSG_RFID_DENSITY                     "密度："
#define MSG_RFID_SPOOL_LENGHT                "后台打印长度："

// Firmware Test
#define MSG_FWTEST_YES                       "把Y命令去下一个"
#define MSG_FWTEST_NO                        "把N命令去下一个"
#define MSG_FWTEST_YES_NO                    "把Y或N命令转到下一个"
#define MSG_FWTEST_ENDSTOP_ERR               "挡块错误！检查电线和连接"
#define MSG_FWTEST_PRESS                     "按住挡块"
#define MSG_FWTEST_INVERT                    "逆向价值"
#define MSG_FWTEST_XAXIS                     "有喷嘴移动到右边？"
#define MSG_FWTEST_YAXIS                     "有嘴前移？"
#define MSG_FWTEST_ZAXIS                     "已喷嘴感动了？"
#define MSG_FWTEST_01                        "手动移动轴X，Y和Z从挡块离开"
#define MSG_FWTEST_02                        "你要检查挡块？"
#define MSG_FWTEST_03                        "开始检查挡块"
#define MSG_FWTEST_04                        "开始检查电机"
#define MSG_FWTEST_ATTENTION                 "注意！检查三个轴是从挡块5 mm以上！"
#define MSG_FWTEST_END                       "完成测试。禁用固件测试和编译"
#define MSG_FWTEST_INTO                      "变化"
#define MSG_FWTEST_ERROR                     "错误"
#define MSG_FWTEST_OK                        " OK "
#define MSG_FWTEST_NDEF                      "没有定义"

#endif // LANGUAGE_CN_H
